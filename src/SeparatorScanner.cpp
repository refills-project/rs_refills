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


#include <refills_msgs/SeparatorArray.h>

#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>

using namespace uima;


class SeparatorScanner : public DrawingAnnotator
{

private:
  ros::NodeHandle nh_;
  tf::StampedTransform camToWorld_;
  sensor_msgs::CameraInfo camInfo_;
  tf::TransformListener *listener;

  ros::Subscriber separatorSubscriber_;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr separatorPoints_;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_, dispCloud_;

  std::mutex lockSeparator_;

  std::string localFrameName_;

  cv::Mat rgb_;
public:

  SeparatorScanner(): DrawingAnnotator(__func__), nh_("~")
  {
    separatorPoints_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
    cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
    dispCloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();

    listener = new tf::TransformListener(nh_, ros::Duration(10.0));
  }

  void separatorAggregator(const refills_msgs::SeparatorArrayPtr &msg)
  {
    std::lock_guard<std::mutex> lock(lockSeparator_);
    for(auto m : msg->separators)
    {
      tf::Stamped<tf::Pose> poseStamped, poseBase;
      tf::poseStampedMsgToTF(m.separator_pose, poseStamped);
      try
      {
        listener->transformPose(localFrameName_, poseStamped.stamp_, poseStamped, poseStamped.frame_id_, poseBase);

        pcl::PointXYZRGBA pt;
        pt.x = poseBase.getOrigin().x();
        pt.y = poseBase.getOrigin().y();
        pt.z = poseBase.getOrigin().z();

        separatorPoints_->points.push_back(pt);
      }
      catch(tf::TransformException ex)
      {
        outWarn(ex.what());
      }
    }
  }

  bool enforceZAxesSimilarity(const pcl::PointXYZRGBA &point_a, const pcl::PointXYZRGBA &point_b, float squared_distance)
  {
    if(fabs(point_a.x - point_b.x) < 0.03f)
      return (true);
    else
      return (false);
  }

  void clusterPoints(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,  pcl::IndicesClustersPtr &clusters)
  {
    outInfo("started clustering");
    pcl::ConditionalEuclideanClustering<pcl::PointXYZRGBA> cec(true);

    cec.setInputCloud(cloud);
    cec.setConditionFunction(boost::bind(&SeparatorScanner::enforceZAxesSimilarity, this, _1, _2, _3));
    cec.setClusterTolerance(2.0);
    cec.setMinClusterSize(cloud->points.size() / 10);
    cec.setMaxClusterSize(cloud->points.size() / 2);
    cec.segment(*clusters);
    //cec.getRemovedClusters(small_clusters, large_clusters);
  }

  void createResultsAndToCas(CAS &tcas,
                             const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
                             const pcl::IndicesClustersPtr &clusters)
  {
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    int idx = 0;
    for(auto c : *clusters)
    {
      rs::Cluster hyp = rs::create<rs::Cluster>(tcas);
      rs::Detection detection = rs::create<rs::Detection>(tcas);

      detection.source.set("SeparatorScanner");


      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cloud, c, centroid);

      tf::Stamped<tf::Pose> pose;
      pose.setOrigin(tf::Vector3(centroid[0], centroid[1], centroid[2]));
      pose.setRotation(tf::Quaternion(0, 0, 0, 1));
      pose.frame_id_ = localFrameName_;
      uint64_t ts = scene.timestamp();

      detection.name.set("separator#" + std::to_string(idx++));
      pose.stamp_ = ros::Time().fromNSec(ts);
      rs::PoseAnnotation poseAnnotation  = rs::create<rs::PoseAnnotation>(tcas);
      poseAnnotation.source.set("SeparatorScanner");
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
      queryAsString = query.asJson();
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
            if(!boost::iequals(objType, "separator"))
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
            separatorSubscriber_.shutdown();
            reset = true;
          }
          if(command == "start")
          {
            separatorSubscriber_ = nh_.subscribe("/separator_marker_detector_node/data_out", 50, &SeparatorScanner::separatorAggregator, this);
          }
        }
      }
    }

    if(reset)
    {
      outInfo("STOP received!");

      pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters);
      clusterPoints(separatorPoints_, clusters);
      createResultsAndToCas(tcas, separatorPoints_, clusters);

      dispCloud_->points.clear();
      for(auto p : separatorPoints_->points)
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

      separatorPoints_->points.clear();
      localFrameName_ = "";
    }
    else
    {
      outInfo("separators found: :" << separatorPoints_->points.size());
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
MAKE_AE(SeparatorScanner)
