#include <uima/api.hpp>


#include <rs/types/all_types.h>
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>
#include <rs/utils/common.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>


#include <refills_msgs/Barcode.h>

#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>

using namespace uima;


class BarcodeScanner : public DrawingAnnotator
{

private:
  ros::NodeHandle nh_;
  tf::StampedTransform camToWorld_;
  sensor_msgs::CameraInfo camInfo_;
  tf::TransformListener *listener;

  ros::Subscriber barcodeSubscriber_;
  std::vector<std::string> barcodes;

  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr barcodePoints_;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_, dispCloud_;

  std::mutex lockBarcode_;

  std::string localFrameName_;

  cv::Mat rgb_;
public:

  BarcodeScanner(): DrawingAnnotator(__func__), nh_("~")
  {
    barcodePoints_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBL>>();
    cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
    dispCloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();

    listener = new tf::TransformListener(nh_, ros::Duration(10.0));
  }

  void barcodeAggregator(const refills_msgs::BarcodePtr &msg)
  {
    std::lock_guard<std::mutex> lock(lockBarcode_);
    tf::Stamped<tf::Pose> poseStamped, poseBase;
    tf::poseStampedMsgToTF(msg->barcode_pose, poseStamped);
    try
    {
      listener->transformPose(localFrameName_, poseStamped.stamp_, poseStamped, poseStamped.frame_id_, poseBase);
      pcl::PointXYZRGBL pt;
      pt.x = poseBase.getOrigin().x();
      pt.y = poseBase.getOrigin().y();
      pt.z = poseBase.getOrigin().z();
      std::vector<std::string>::iterator it = std::find(barcodes.begin(), barcodes.end(), msg->barcode);
      if(it != barcodes.end())
      {
        pt.label = it - barcodes.begin();
      }
      else
      {
        pt.label = barcodes.size();
        barcodes.push_back(msg->barcode);
      }
      barcodePoints_->points.push_back(pt);
    }
    catch(tf::TransformException ex)
    {
      outWarn(ex.what());
    }
  }

  bool enforceSimilarity(const pcl::PointXYZRGBL &point_a, const pcl::PointXYZRGBL &point_b, float squared_distance)
  {
    if(fabs(point_a.x - point_b.x) < 0.03f && point_a.label == point_b.label)
      return (true);
    else
      return (false);
  }

  void clusterPoints(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr &cloud,  pcl::IndicesClustersPtr &clusters)
  {
    outInfo("started clustering");
    pcl::ConditionalEuclideanClustering<pcl::PointXYZRGBL> cec(true);

    cec.setInputCloud(cloud);
    cec.setConditionFunction(boost::bind(&BarcodeScanner::enforceSimilarity, this, _1, _2, _3));
    cec.setClusterTolerance(2.0);
    cec.setMinClusterSize(cloud->points.size() / 10);
    cec.setMaxClusterSize(cloud->points.size() / 2);
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
    for(auto c : *clusters)
    {
      rs::ObjectHypothesis hyp = rs::create<rs::ObjectHypothesis>(tcas);
      rs::Detection detection = rs::create<rs::Detection>(tcas);

      detection.source.set("BarcodeScanner");


      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cloud, c, centroid);

      tf::Stamped<tf::Pose> pose;
      pose.setOrigin(tf::Vector3(centroid[0], centroid[1], centroid[2]));
      pose.setRotation(tf::Quaternion(0, 0, 0, 1));
      pose.frame_id_ = localFrameName_;
      uint64_t ts = scene.timestamp();

      //TODO:: take point label from cluster;
      std::string barcodeLabel = barcodes[cloud->points[c.indices[0]].label];
      detection.name.set("barcode#" + barcodeLabel);
      pose.stamp_ = ros::Time().fromNSec(ts);
      rs::PoseAnnotation poseAnnotation  = rs::create<rs::PoseAnnotation>(tcas);
      poseAnnotation.source.set("BarcodeScanner");
      poseAnnotation.world.set(rs::conversion::to(tcas, pose));
      poseAnnotation.camera.set(rs::conversion::to(tcas, pose));

      hyp.annotations.append(detection);
      hyp.annotations.append(poseAnnotation);
      scene.identifiables.append(hyp);
    }
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");

    return UIMA_ERR_NONE;
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    MEASURE_TIME;
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);

    cas.get(VIEW_CLOUD, *cloud_);
    cas.get(VIEW_COLOR_IMAGE, rgb_);
    cas.get(VIEW_CAMERA_INFO, camInfo_);

    std::string queryAsString = "";
    rs::Query query = rs::create<rs::Query>(tcas);

    //see if query has what we want;
    bool reset = false;
    if(cas.getFS("QUERY", query))
    {
      queryAsString = query.query();
      if(queryAsString != "")
      {
        rapidjson::Document jsonQuery;
        jsonQuery.Parse(queryAsString.c_str());

        if(jsonQuery.HasMember("scan"))
        {
          if(jsonQuery["scan"].HasMember("location"))
            localFrameName_ = jsonQuery["scan"]["location"].GetString();
          if(jsonQuery["scan"].HasMember("type"))
          {
            std::string objType = jsonQuery["scan"]["type"].GetString();
            if(!boost::iequals(objType, "barcode"))
            {
              outInfo("Asking to scan an object that is not a separator. Returning;");
              return UIMA_ERR_NONE;
            }
          }
          else
            return UIMA_ERR_NONE;

        }
        if(jsonQuery["scan"].HasMember("command"))
        {
          std::string command = jsonQuery["scan"]["command"].GetString();
          if(command == "stop")
          {
            barcodeSubscriber_.shutdown();
            reset = true;
          }
          if(command == "start")
          {
            barcodeSubscriber_ = nh_.subscribe("/barcode/pose", 50, &BarcodeScanner::barcodeAggregator, this);
          }
        }
      }
    }

    if(reset)
    {
      outInfo("STOP received!");

      pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters);
      clusterPoints(barcodePoints_, clusters);
      createResultsAndToCas(tcas, barcodePoints_, clusters);

      dispCloud_->points.clear();
      for(auto p : barcodePoints_->points)
      {
        pcl::PointXYZRGBA pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = p.z;
        dispCloud_->points.push_back(pt);
      }

      outInfo("Clusters found:" << clusters->size());
      for(int i = 0; i < clusters->size(); ++i)
      {
        outInfo((*clusters)[i].indices.size());
        for(int j = 0; j < (*clusters)[i].indices.size(); ++j)
        {
          dispCloud_->points[(*clusters)[i].indices[j]].rgba = rs::common::colors[i % clusters->size()];
        }
      }

      barcodePoints_->points.clear();
      localFrameName_ = "";
    }
    else
    {
      outInfo("separators found: :" << barcodePoints_->points.size());
    }

    return UIMA_ERR_NONE;
  }

  void drawImageWithLock(cv::Mat &disp)
  {

  }
  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {

  }
};
// This macro exports an entry point that is used to create the annotator.
MAKE_AE(BarcodeScanner)
