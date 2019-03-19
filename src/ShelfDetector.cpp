#include <uima/api.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <mutex>

#include <tf_conversions/tf_eigen.h>
#include <rs_refills/common_structs.h>
#include <tf/tf.h>

#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl/io/pcd_io.h>

#include <pcl/common/transforms.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <rs/types/all_types.h>
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/utils/common.h>
#include <rs/io/TFListenerProxy.h>
#include <rs/DrawingAnnotator.h>

#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>

#include <refills_msgs/Barcode.h>
#include <refills_msgs/SeparatorArray.h>


using namespace uima;

/**
  DEPRECATED
 * @brief The ShelfDetector Annotator
 *  Detects shelves in a supermarket: assumes a localized robot and a point cloud
 *  that has been filtered to contain points only from the current shelf meter.
 *  paramterest to tweak:
 *   - distance between two lines
 *   - variance on y of a line
 *   - number of inliers in a line needs be fairly big (depends on voxelizaiton param)
 *   - max height of a shelf_system can be 1.85
 */
class ShelfDetector : public DrawingAnnotator
{

public:
  //ROS interface related members
  ros::NodeHandle nh_;
  tf::StampedTransform camToWorld_;
  sensor_msgs::CameraInfo camInfo_;

  rs::TFListenerProxy listener_;
  ros::Subscriber barcodeSubscriber_;
  ros::Subscriber separatorSubscriber_;


  //RoboSherlock related conatiners of raw data
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_, dispCloud_;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered_;
  pcl::PointCloud<pcl::Normal>::Ptr normals_;
  cv::Mat mask_, rgb_, disp_, bin_, grey_;

  //algorithm related members
  std::vector<pcl::PointIndices> label_indices_;
  std::vector<pcl::PointIndicesPtr> line_inliers_;
  std::vector<Eigen::VectorXf> line_models_;
  std::mutex lockBarcode_, lockSeparator_;

  struct Line {
    pcl::PointXYZRGBA pt_begin;
    pcl::PointXYZRGBA pt_end;
    uint8_t id;
  };

  std::vector<Line> lines_;

  std::vector<tf::Stamped<tf::Pose>> barcodePoses_;
  std::vector<tf::Stamped<tf::Pose>> separatorPoses_;

  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr barcodePoints_, separatorPoints_;

  //visualization related members
  enum class DisplayMode {
    COLOR,
    EDGE,
    BINARY,
    GREY
  } dispMode;

  //other
  std::string localFrameName_;
  float shelfmaxInlierDistance_;
public:

  ShelfDetector(): DrawingAnnotator(__func__), nh_("~"), dispMode(DisplayMode::COLOR),shelfmaxInlierDistance_(0.08)
  {
    cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
    dispCloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
    cloud_filtered_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();

    barcodePoints_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBL>>();
    separatorPoints_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBL>>();

    normals_ = boost::make_shared<pcl::PointCloud<pcl::Normal>>();
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    ctx.extractValue("shelf_max_inlier_distance", shelfmaxInlierDistance_);
    setAnnotatorContext(ctx);
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  void barcodeAggregator(const refills_msgs::BarcodePtr &msg)
  {

    std::lock_guard<std::mutex> lock(lockBarcode_);
    tf::Stamped<tf::Pose> poseStamped, poseBase;
    tf::poseStampedMsgToTF(msg->barcode_pose, poseStamped);
    try {
      listener_.listener->waitForTransform(localFrameName_, poseStamped.frame_id_, poseStamped.stamp_, ros::Duration(1.0));
      listener_.listener->transformPose(localFrameName_, poseStamped.stamp_, poseStamped, poseStamped.frame_id_, poseBase);
      pcl::PointXYZRGBL pt;
      pt.x = poseBase.getOrigin().x();
      pt.y = poseBase.getOrigin().y();
      pt.z = poseBase.getOrigin().z();
      pt.label = 1;
      barcodePoints_->points.push_back(pt);
    }
    catch(tf::TransformException ex) {
      outWarn(ex.what());
    }
  }

  void separatorAggregator(const refills_msgs::SeparatorArrayPtr &msg)
  {
    std::lock_guard<std::mutex> lock(lockSeparator_);
    for(auto m : msg->separators) {
      tf::Stamped<tf::Pose> poseStamped, poseBase;
      tf::poseStampedMsgToTF(m.separator_pose, poseStamped);
      try {
        listener_.listener->waitForTransform(localFrameName_, poseStamped.frame_id_, poseStamped.stamp_, ros::Duration(1.0));
        listener_.listener->transformPose(localFrameName_, poseStamped.stamp_, poseStamped, poseStamped.frame_id_, poseBase);

        pcl::PointXYZRGBL pt;
        pt.x = poseBase.getOrigin().x();
        pt.y = poseBase.getOrigin().y();
        pt.z = poseBase.getOrigin().z();
        pt.label = 0;
        separatorPoints_->points.push_back(pt);
      }
      catch(tf::TransformException ex) {
        outWarn(ex.what());
      }
    }
  }

  void clear()
  {
    //this is an overkill here, but good to learn about RAII way
    std::unique_lock<std::mutex> lk1(lockBarcode_, std::defer_lock);
    std::unique_lock<std::mutex> lk2(lockSeparator_, std::defer_lock);
    std::lock(lk1, lk2);
    barcodePoses_.clear();
    separatorPoses_.clear();
    barcodePoints_->points.clear();
    separatorPoints_->points.clear();
  }

  bool callbackKey(const int key, const Source source)
  {
    switch(key) {
    case 'c':
      dispMode = DisplayMode::COLOR;
      return true;
    case 'e':
      dispMode = DisplayMode::EDGE;
      return true;
    case 'b':
      dispMode = DisplayMode::BINARY;
      return true;
    case 'g':
      dispMode = DisplayMode::GREY;
      return true;

    }
    return false;
  }

  bool enforceZAxesSimilarity(const pcl::PointXYZRGBL &point_a, const pcl::PointXYZRGBL &point_b, float squared_distance)
  {
    if(fabs(point_a.z - point_b.z) < shelfmaxInlierDistance_)
      return (true);
    else
      return (false);
  }

  void clusterPoints(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr &cloud,  pcl::IndicesClustersPtr &clusters)
  {
    outInfo("started clustering");

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBL> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(30);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud);


    pcl::ConditionalEuclideanClustering<pcl::PointXYZRGBL> cec(true);

    cec.setInputCloud(cloud);
    cec.setConditionFunction(boost::bind(&ShelfDetector::enforceZAxesSimilarity, this, _1, _2, _3));
    cec.setClusterTolerance(2.0);
    cec.setMinClusterSize(10);
    cec.setMaxClusterSize(cloud->points.size());
    cec.segment(*clusters);
    //cec.getRemovedClusters(small_clusters, large_clusters);
  }

  void createResultsAndToCas(CAS &tcas,
                             const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr &cloud,
                             const pcl::IndicesClustersPtr &clusters)
  {
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    int idx = 0;
    for(auto c : *clusters) {
      rs::ObjectHypothesis hyp = rs::create<rs::ObjectHypothesis>(tcas);
      rs::Detection detection = rs::create<rs::Detection>(tcas);

      detection.source.set("ShelfDetector");

      uint64_t ts = static_cast<uint64_t>(scene.timestamp());

      pcl::PointIndices barcodeIndices;
      pcl::PointIndices separatorIndices;
      int bCount = 0, sCount = 0 ;
      std::for_each(c.indices.begin(), c.indices.end(), [&cloud, &bCount, &sCount, &barcodeIndices, &separatorIndices](int n) {
        if(cloud->points[n].label == 1) {
          bCount++;
          barcodeIndices.indices.push_back(n);
          //this is dangerous...there could be a shelf where we don't see any separators...
        }
        else{
            separatorIndices.indices.push_back(n);
          sCount++;
        }
      });

      std::string layerType = "standing";


      if(sCount == 0 || (sCount != 0 && bCount != 0 && (float)sCount / (float)bCount < 0.20))
        layerType = "rack";

      outInfo("Separator in cluster: "<<sCount<<" Barcode In Cluster: "<<bCount<< " Ratio: "<< sCount/static_cast<float>(bCount)<<" is of type: "<<layerType<<"#"<<idx);
//       outInfo("Separator in cluster: "<<separatorIndices.indices.size()<<" Barcode In Cluster: "<<barcodeIndices.indices.size()<< " Ratio: "<< sCount/static_cast<float>(bCount)<<" is of type: "<<layerType<<"#"<<idx);
      detection.name.set(layerType + "#" + std::to_string(idx++));


      Eigen::Vector4f centroid,centroidSep,centroidBar;
      if(!barcodeIndices.indices.empty() && separatorIndices.indices.empty())
      {

          pcl::compute3DCentroid(*cloud, barcodeIndices,centroidBar);
          pcl::compute3DCentroid(*cloud, separatorIndices,centroidSep);
          centroid= (centroidBar + centroidSep)/2;
      }
      else{
      pcl::compute3DCentroid(*cloud, c, centroid);}

      tf::Stamped<tf::Pose> pose;
      pose.setOrigin( tf::Vector3(static_cast<double>(centroid[0]), static_cast<double>(centroid[1]), static_cast<double>(centroid[2])));
      pose.setRotation(tf::Quaternion(0, 0, 0, 1));
      pose.frame_id_ = localFrameName_;
      pose.stamp_ = ros::Time().fromNSec(ts);

      rs::PoseAnnotation poseAnnotation  = rs::create<rs::PoseAnnotation>(tcas);
      poseAnnotation.source.set("ShelfDetector");
      poseAnnotation.world.set(rs::conversion::to(tcas, pose));
      poseAnnotation.camera.set(rs::conversion::to(tcas, pose));

      hyp.annotations.append(detection);
      hyp.annotations.append(poseAnnotation);
      scene.identifiables.append(hyp);

    }
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    MEASURE_TIME;
    label_indices_.clear();
    line_inliers_.clear();

    rs::SceneCas cas(tcas);
    cas.get(VIEW_CLOUD, *cloud_);
    cas.get(VIEW_COLOR_IMAGE, rgb_);
    cas.get(VIEW_CAMERA_INFO, camInfo_);

    std::string queryAsString = "";
    rs::Query query = rs::create<rs::Query>(tcas);

    //see if query has what we want;
    bool reset = false;
    if(cas.getFS("QUERY", query)) {

      queryAsString = query.query();
      outInfo("query as string: " << queryAsString);
      if(queryAsString != "") {
        rapidjson::Document jsonQuery;
        jsonQuery.Parse(queryAsString.c_str());

        if(jsonQuery.HasMember("scan")) {
          if(jsonQuery["scan"].HasMember("location"))
            localFrameName_ = jsonQuery["scan"]["location"].GetString();
          if(jsonQuery["scan"].HasMember("type")) {
            std::string objType = jsonQuery["scan"]["type"].GetString();
            if(!boost::iequals(objType, "shelf")) {
              outInfo("Asking to scan an object that is not a shelf. Returning;");
              return UIMA_ERR_NONE;
            }
          }
          else
            return UIMA_ERR_NONE;

        }
        if(jsonQuery["scan"].HasMember("command")) {
          std::string command = jsonQuery["scan"]["command"].GetString();
          if(command == "stop") {
            outWarn("Stopping Subscribers");
            barcodeSubscriber_.shutdown();
            separatorSubscriber_.shutdown();
            reset = true;
          }
          if(command == "start") {
            outWarn("Starting subscribers!");
            barcodeSubscriber_ = nh_.subscribe("/barcode/pose", 50, &ShelfDetector::barcodeAggregator, this);
            separatorSubscriber_ = nh_.subscribe("/separator_marker_detector_node/data_out", 50, &ShelfDetector::separatorAggregator, this);
          }
        }
      }
    }

    rs::Scene scene = cas.getScene();

    if(reset) {
      outInfo("STOP received!");
      outInfo("Final coung: [barcodes]: " << barcodePoints_->size() << " [separators]:" << separatorPoints_->size());

      pcl::PointCloud<pcl::PointXYZRGBL>::Ptr concatCloud(new pcl::PointCloud<pcl::PointXYZRGBL>());
      *concatCloud = *barcodePoints_;
      *concatCloud += *separatorPoints_;
      concatCloud->height = 1;
      concatCloud->width = concatCloud->points.size();
      outInfo("concatenated cloud size: " << concatCloud->size());

      pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters);
      clusterPoints(concatCloud, clusters);
      createResultsAndToCas(tcas, concatCloud, clusters);

      dispCloud_->points.clear();
      for(auto p : concatCloud->points) {
        pcl::PointXYZRGBA pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = p.z;
        pt.rgba = 0xffffffff;
        //        pt.a = 255;
        dispCloud_->points.push_back(pt);
      }

      outInfo("Clusters found:" << clusters->size());
      for(int i = 0; i < clusters->size(); ++i) {
        outInfo((*clusters)[i].indices.size());
        for(int j = 0; j < (*clusters)[i].indices.size(); ++j) {
          dispCloud_->points[(*clusters)[i].indices[j]].rgba = rs::common::colors[i % clusters->size()];
          dispCloud_->points[(*clusters)[i].indices[j]].a = 255;
        }
      }
      clear();
      localFrameName_ = "";
    }
    else {
      outInfo("barcodes found: :" << barcodePoints_->points.size());
      outInfo("separators found: :" << separatorPoints_->points.size());
    }

    return UIMA_ERR_NONE;
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    switch(dispMode) {
    case DisplayMode::COLOR:
      disp = rgb_.clone();
      break;
    case DisplayMode::EDGE:
      disp = disp_.clone();
      break;
    case DisplayMode::BINARY:
      disp = bin_.clone();
      break;
    case DisplayMode::GREY:
      disp = grey_.clone();
      break;
    }
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    const std::string &cloudname = "cloud";
    double pointSize = 4.0;
    double pointSize2 = pointSize / 4.0;

    int idx = 0;
    visualizer.removeAllShapes();

    if(firstRun) {
      visualizer.addPointCloud(dispCloud_, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    else {
      visualizer.updatePointCloud(dispCloud_, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(ShelfDetector)
