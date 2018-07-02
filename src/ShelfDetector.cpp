#include <uima/api.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <mutex>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include <pcl/point_types.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/filters/passthrough.h>

#include <pcl/features/boundary.h>
#include <pcl/features/organized_edge_detection.h>

#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include <pcl/ModelCoefficients.h>

#include <rs/types/all_types.h>
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/utils/common.h>
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
  tf::TransformListener *listener;
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

  int min_line_inliers_;
  float max_variance_;

  std::mutex lockBarcode_, lockSeparator_;

  struct Line
  {
    pcl::PointXYZRGBA pt_begin;
    pcl::PointXYZRGBA pt_end;
    uint8_t id;
  };

  std::vector<Line> lines_;

  std::vector<tf::Stamped<tf::Pose>> barcodePoses_;
  std::vector<tf::Stamped<tf::Pose>> separatorPoses_;

  //visualization related members
  enum class DisplayMode
  {
    COLOR,
    EDGE,
    BINARY,
    GREY
  } dispMode;

  //other
  std::string localFrameName_;
public:

  ShelfDetector(): DrawingAnnotator(__func__), nh_("~"), min_line_inliers_(50), max_variance_(0.01), dispMode(DisplayMode::EDGE)
  {
    cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
    dispCloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
    cloud_filtered_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();

    normals_ = boost::make_shared<pcl::PointCloud<pcl::Normal>>();

    barcodeSubscriber_ = nh_.subscribe("what is the topic name?", 50, &ShelfDetector::barcodeAggregator, this);
    separatorSubscriber_ = nh_.subscribe("what is the topic name?", 50, &ShelfDetector::separatorAggregator, this);

    listener = new tf::TransformListener(nh_, ros::Duration(10.0));
  }

  void barcodeAggregator(const refills_msgs::BarcodePtr &msg)
  {
    tf::Stamped<tf::Pose> poseStamped;
    tf::poseStampedMsgToTF(msg->barcode_pose, poseStamped);
    barcodePoses_.push_back(poseStamped);
  }

  void separatorAggregator(const refills_msgs::SeparatorArrayPtr &msg)
  {
    for(auto m : msg->separators)
    {
      tf::Stamped<tf::Pose> poseStamped;
      tf::poseStampedMsgToTF(m.separator_pose, poseStamped);
      separatorPoses_.push_back(poseStamped);
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
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    ctx.extractValue("min_line_inliers", min_line_inliers_);
    ctx.extractValue("max_variance", max_variance_);
    setAnnotatorContext(ctx);
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  void projectPointOnPlane(pcl::PointXYZRGBA &pt, const std::vector<float> &plane_model)
  {
    assert(plane_model.size() == 4);
    cv::Point3f normal(plane_model[0], plane_model[1], plane_model[2]);
    float planeDist = plane_model[3];
    cv::Point3f point(pt.x, pt.y, pt.z);
    float pointDist = point.dot(normal);
    float t = planeDist + pointDist;
    cv::Point3f projected = point - normal * t;
    pt.x = projected.x;
    pt.z = projected.z;
    pt.y = projected.y;
  }

  void projectPointCloudOnPlane(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const std::vector<float> &plane_model)
  {
    for(auto &p : cloud->points)
      projectPointOnPlane(p, plane_model);
  }

  void solveLineIds()
  {
    for(auto inliers : line_inliers_)
    {
      Line line;
      line.pt_begin = cloud_filtered_->points[inliers->indices[0]];
      line.pt_begin = cloud_filtered_->points[inliers->indices[0]];
      std::for_each(inliers->indices.begin() + 1, inliers->indices.end(), [&line, this](int n)
      {
        if(this->cloud_filtered_->points[n].x < line.pt_begin.x)
        {
          line.pt_begin = this->cloud_filtered_->points[n];
        }

        if(this->cloud_filtered_->points[n].x > line.pt_end.x)
        {
          line.pt_end = this->cloud_filtered_->points[n];
        }
      });
      bool found = false;
      for(auto &l : lines_)
      {
        double distb = rs::common::pointToPointDistance2DSqrt(l.pt_begin.y, l.pt_begin.z,  line.pt_begin.y, line.pt_begin.z);
        double diste = rs::common::pointToPointDistance2DSqrt(l.pt_end.y, l.pt_end.z,  line.pt_end.y, line.pt_end.z);
        if(distb < 0.1 && diste < 0.1)
        {
          found = true;
          l.pt_begin.x = (l.pt_begin.x + line.pt_begin.x) / 2;
          l.pt_begin.y = (l.pt_begin.y + line.pt_begin.y) / 2;
          l.pt_begin.z = (l.pt_begin.z + line.pt_begin.z) / 2;

          l.pt_end.x = (l.pt_end.x + line.pt_end.x) / 2;
          l.pt_end.y = (l.pt_end.y + line.pt_end.y) / 2;
          l.pt_end.z = (l.pt_end.z + line.pt_end.z) / 2;

          break;
        }
      }

      if(!found && line.pt_begin.z < 1.85)
      {
        line.id = lines_.size();
        lines_.push_back(line);
      }
    }
  }

  void addToCas(CAS &tcas)
  {
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    for(auto line : lines_)
    {
      rs::Cluster hyp = rs::create<rs::Cluster>(tcas);
      rs::Detection detection = rs::create<rs::Detection>(tcas);
      detection.source.set("ShelfDetector");
      detection.name.set(std::to_string(line.id));
      tf::Stamped<tf::Pose> pose;
      pose.setOrigin(tf::Vector3(line.pt_begin.x, line.pt_begin.y, line.pt_begin.z));
      pose.setRotation(tf::Quaternion(0, 0, 0, 1));
      pose.frame_id_ = "map";
      uint64_t ts = scene.timestamp();
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

  void makeMaskedImage()
  {
    mask_ = rgb_.clone();
    for(int i = 0; i < cloud_filtered_->points.size(); ++i)
    {
      if(!pcl::isFinite(cloud_filtered_->points[i]))
      {
        cv::Vec3f color(0, 0, 0);
        mask_.at<cv::Vec3b>(i) = color;
      }
    }
  }

  void findLinesInImage()
  {
    cv::Mat edge, dilatedCanny;
    cv::cvtColor(mask_, grey_, cv::COLOR_BGR2GRAY);

    cv::threshold(grey_, bin_, 150, 255, cv::THRESH_BINARY);
    cv::Canny(bin_, edge, 50, 150);
    cv::Mat element = getStructuringElement(cv::MORPH_CROSS,
                                            cv::Size(3, 3),
                                            cv::Point(2, 2));
    cv::dilate(edge, dilatedCanny, element);


    disp_ = dilatedCanny.clone();

    std::vector<cv::Vec4i> linesP; // will hold the results of the detection
    cv::HoughLinesP(dilatedCanny, linesP, 1, CV_PI / 180, 50, 400, 15); // runs the actual detection
    // Draw the lines
    for(size_t i = 0; i < linesP.size(); i++)
    {
      cv::Vec4i l = linesP[i];
      cv::line(rgb_, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
    }

  }

  bool callbackKey(const int key, const Source source)
  {
    switch(key)
    {
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

  bool findLinesInCloud()
  {
    pcl::OrganizedEdgeFromNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Label> oed;
    oed.setInputNormals(normals_);
    oed.setInputCloud(cloud_filtered_);
    oed.setDepthDisconThreshold(0.05);
    oed.setMaxSearchNeighbors(0.03);
    oed.setEdgeType(oed.EDGELABEL_NAN_BOUNDARY);
    pcl::PointCloud<pcl::Label> labels;

    oed.compute(labels, label_indices_);
    if(label_indices_[0].indices.size() == 0)
    {
      outWarn("No NaN boundaries found. Exiting annotator");
      return false;
    }

    pcl::ExtractIndices<pcl::PointXYZRGBA> ei;
    outInfo("Before filter: " << cloud_filtered_->points.size());
    ei.setInputCloud(cloud_filtered_);
    //this is the bullshit of PCL...one algo returns PointIndices next algo want a f'in pointer
    ei.setIndices(boost::make_shared<pcl::PointIndices>(label_indices_[0]));
    ei.setKeepOrganized(true);
    ei.filterDirectly(cloud_filtered_);

    pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
    vg.setInputCloud(cloud_filtered_);
    vg.setLeafSize(0.02, 0.02, 0.02);
    vg.filter(*cloud_filtered_);

    //    pcl::PointCloud <pcl::PointXYZRGBA>::Ptr edge_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud <pcl::PointXYZRGBA>::Ptr edge_cloud = cloud_filtered_->makeShared();

    std::vector<float> xz_plane{0.0, 1.0, 0.0, 0.5};
    projectPointCloudOnPlane(edge_cloud, xz_plane);

    //TODO what should be a stop criteria here?
    int count = 0;
    int remaining_points = edge_cloud->size();
    while(count++ < 5 && remaining_points > 0)
    {

      //lines parallel to the X-AXES (THIS CAN CHANGE)
      pcl::SampleConsensusModelParallelLine<pcl::PointXYZRGBA>::Ptr
      model_pl(new pcl::SampleConsensusModelParallelLine<pcl::PointXYZRGBA> (edge_cloud));
      model_pl->setAxis(Eigen::Vector3f(1.0, 0, 0));

      outInfo("edge_cloud.size: " << edge_cloud->size());
      pcl::search::Search<pcl::PointXYZRGBA>::Ptr search(new pcl::search::KdTree<pcl::PointXYZRGBA>);
      search->setInputCloud(edge_cloud);
      outInfo("kdTree created");
      model_pl->setSamplesMaxDist(0.07, search);
      model_pl->setEpsAngle(1.5 * M_PI / 180);

      pcl::RandomSampleConsensus<pcl::PointXYZRGBA> ransac(model_pl);
      ransac.setDistanceThreshold(0.01);

      ransac.computeModel();
      pcl::PointIndicesPtr inliers(new pcl::PointIndices());
      ransac.getInliers(inliers->indices);

      float avg_y = 0;
      std::for_each(inliers->indices.begin(), inliers->indices.end(), [&avg_y, this](int n)
      {
        avg_y += this->cloud_filtered_->points[n].y;
      }
                   );
      avg_y = avg_y / inliers->indices.size();
      float ssd = 0;
      std::for_each(inliers->indices.begin(), inliers->indices.end(), [avg_y, &ssd, this](int n)
      {
        ssd += (this->cloud_filtered_->points[n].y - avg_y) * (this->cloud_filtered_->points[n].y - avg_y);
      }
                   );

      float var = std::sqrt(ssd / (inliers->indices.size()));


      //the variance on y needs to be small
      if(inliers->indices.size() > min_line_inliers_ && var < max_variance_)
      {
        outInfo("variance is : " << var);
        outInfo("Line inliers found: " << inliers->indices.size());
        Eigen::VectorXf model_coeffs;
        ransac.getModelCoefficients(model_coeffs);
        outInfo("x = " << model_coeffs[0] << " y = " << model_coeffs[1] << " z = " << model_coeffs[2]);
        line_models_.push_back(model_coeffs);
        line_inliers_.push_back(inliers);
      }
      else
      {
        outWarn("variance was: " << var);
        outWarn("inliers was:  " << inliers->indices.size());
      }
      ei.setInputCloud(edge_cloud);
      ei.setIndices(inliers);
      ei.setNegative(true);
      ei.setKeepOrganized(true);
      ei.filterDirectly(edge_cloud);
      remaining_points -= inliers->indices.size();
    }
    return true;
  }

  void filterCloud(const tf::StampedTransform &poseStamped)
  {
    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    float minX, minY, minZ;
    float maxX, maxY, maxZ;

    minX = 0.001 ;
    maxX = minX + 0.98; //1m shelf

    minY = -0.04; //move closer to cam with 2 cm
    maxY = minY + 0.25; //the deepest shelf

    minZ = 0.15 ; //bottom shelf is not interesting
    maxZ = minZ + 1.8; //make sure to get point from the top

    pass.setInputCloud(cloud_);
    pass.setKeepOrganized(true);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(minX, maxX);
    pass.filter(*cloud_filtered_);

    pass.setInputCloud(cloud_filtered_);
    pass.setFilterFieldName("y");//
    pass.setFilterLimits(minY, maxY);
    pass.filter(*cloud_filtered_);

    pass.setInputCloud(cloud_filtered_);
    pass.setFilterFieldName("z");//
    pass.setFilterLimits(minZ, maxZ);
    pass.filter(*cloud_filtered_);

    {
      MEASURE_TIME;
      pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
      sor.setInputCloud(cloud_filtered_);
      sor.setKeepOrganized(true);
      sor.setMeanK(30);
      sor.setStddevMulThresh(0.5);
      sor.filter(*cloud_filtered_);
      outInfo("SOR filter");
    }

    *dispCloud_ = *cloud_filtered_;
  }


  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    MEASURE_TIME;
    label_indices_.clear();
    line_inliers_.clear();

    rs::SceneCas cas(tcas);
    cas.get(VIEW_CLOUD, *cloud_);
    cas.get(VIEW_NORMALS, *normals_);
    cas.get(VIEW_COLOR_IMAGE, rgb_);
    cas.get(VIEW_CAMERA_INFO, camInfo_);

    std::string queryAsString = "";
    rs::Query query = rs::create<rs::Query>(tcas);


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
          {
            localFrameName_ = jsonQuery["scan"]["location"].GetString();
          }
        }
        if(jsonQuery["scan"].HasMember("command"))
        {
          std::string command = jsonQuery["scan"]["command"].GetString();
          if(command == "stop")
          {
            outWarn("Clearing chache of line segments");
            reset = true;
          }
        }
      }
    }

    if(!reset)
    {

      rs::Scene scene = cas.getScene();
      try
      {
        if(localFrameName_ == "map")
        {
          rs::conversion::from(scene.viewPoint.get(), camToWorld_);
        }
        else
        {
          listener->waitForTransform(localFrameName_, camInfo_.header.frame_id, ros::Time(0),/*camInfo_.header.stamp,*/ ros::Duration(2));
          listener->lookupTransform(localFrameName_, camInfo_.header.frame_id, ros::Time(0),/*camInfo_.header.stamp,*/ camToWorld_);
        }
      }
      catch(tf::TransformException &ex)
      {
        outError(ex.what());
        return UIMA_ERR_NONE;
      }
      Eigen::Affine3d eigenTransform;
      tf::transformTFToEigen(camToWorld_, eigenTransform);
      pcl::transformPointCloud<pcl::PointXYZRGBA>(*cloud_, *cloud_, eigenTransform);

      filterCloud(camToWorld_);

      makeMaskedImage();
      findLinesInImage();
      findLinesInCloud();

      solveLineIds();
    }

    //always add to CAS
    addToCas(tcas);

    //suboptimal but f. it
    if(reset)
    {
      lines_.clear();
      localFrameName_ = "";
    }
    return UIMA_ERR_NONE;
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    switch(dispMode)
    {
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
    for(int i = 0; i < line_inliers_.size(); ++i)
    {
      for(int j = 0; j < line_inliers_[i]->indices.size(); ++j)
      {
        cloud_filtered_->points[line_inliers_[i]->indices[j]].rgba = rs::common::colors[i];
        cloud_filtered_->points[line_inliers_[i]->indices[j]].a = 255;
      }
    }

    int idx = 0;
    visualizer.removeAllShapes();
    for(auto line : lines_)
    {
      std::stringstream lineName;
      lineName << "line_" << idx++;
      visualizer.addLine(line.pt_begin, line.pt_end, lineName.str());
      visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4.0, lineName.str());
      visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, lineName.str());

    }

    if(firstRun)
    {
      visualizer.addPointCloud(dispCloud_, "original_filtered");
      visualizer.addPointCloud(cloud_filtered_, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize2, "original_filtered");
    }
    else
    {

      visualizer.updatePointCloud(dispCloud_, "original_filtered");
      visualizer.updatePointCloud(cloud_filtered_, cloudname);//this is very filtered: boundary cloud
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize2, "original_filtered");
    }
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(ShelfDetector)
