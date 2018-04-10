#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>

//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/utils/common.h>
#include <rs/types/all_types.h>
#include <rs/DrawingAnnotator.h>


//image_transport
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

//tf
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>


//rapidjson
#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>

//json_prolog
#include <json_prolog/prolog.h>

using namespace uima;


class ProductCounter : public DrawingAnnotator
{
private:
  bool external_, useLocalFrame_;
  tf::StampedTransform camToWorld_;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudFiltered_;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr_;
  std::vector<pcl::PointIndices> cluster_indices_;

  cv::Mat rgb_;
  std::string localFrameName_;

  struct BoundingBox
  {
    pcl::PointXYZ minPt, maxPt;
  };


  tf::TransformListener *listener;
  std::vector<BoundingBox> cluster_boxes;
  ros::NodeHandle nodeHandle_;

  image_transport::Publisher image_pub_;
  image_transport::ImageTransport it_;

    sensor_msgs::CameraInfo camInfo_;

public:

  ProductCounter(): DrawingAnnotator(__func__), useLocalFrame_(false), nodeHandle_("~"),it_(nodeHandle_)
  {
    cloudFiltered_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
    cloud_ptr_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();

    listener = new tf::TransformListener(nodeHandle_, ros::Duration(10.0));

    image_pub_ = it_.advertise("counting_image", 1, true);

  }
  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    ctx.extractValue("external", external_);

    ctx.extractValue("use_local_frame", useLocalFrame_);
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  void countWithExternalAlgo(CAS &tcas)
  {
    outWarn("Not yet implemented");
  }

  bool handleQuery(CAS &tcas, std::string &obj, tf::Stamped<tf::Pose> &pose, std::string &shelf_type, float &distToNextSep)
  {
    rs::SceneCas cas(tcas);
    rs::Query query = rs::create<rs::Query>(tcas);
    if(cas.getFS("QUERY", query))
    {
      std::string queryAsString = query.asJson();
      if(queryAsString != "")
      {
        rapidjson::Document doc;
        doc.Parse(queryAsString.c_str());
        if(doc.HasMember("detect"))
        {
          rapidjson::Value &dQuery = doc["detect"];

          if(dQuery.HasMember("type"))
          {
            obj = dQuery["type"].GetString();
          }
          if(dQuery.HasMember("pose_stamped"))
          {

            tf::Vector3 position;
            position.setX(dQuery["pose_stamped"]["pose"]["position"]["x"].GetFloat());
            position.setY(dQuery["pose_stamped"]["pose"]["position"]["y"].GetFloat());
            position.setZ(dQuery["pose_stamped"]["pose"]["position"]["z"].GetFloat());
            pose.frame_id_ = dQuery["pose_stamped"]["header"]["frame_id"].GetString();
            pose.setOrigin(position);
            pose.setRotation(tf::Quaternion(0, 0, 0, 1));
            //            pose.stamp_ = ros::Time::now();
          }
          else
            return false;
          if(useLocalFrame_)
          {
            if(!dQuery.HasMember("location")) return false;
            localFrameName_ = dQuery["location"].GetString();
          }

          if(dQuery.HasMember("shelf_type")) shelf_type = dQuery["shelf_type"].GetString();
          else shelf_type = "standing";

          if(dQuery.HasMember("width")) distToNextSep = dQuery["width"].GetFloat();
          else distToNextSep = 0.0;
        }
        else
          return false;
      }
      return true;
    }
    return false;
  }

  bool getObjectDims(const std::string obj,
                     double &height, double &width, double &depth)
  {
    //owl_instance_from_class(shop:'ProductWithAN377954',I),object_dimensions(I,D,W,H).
    std::stringstream plQuery;
    std::stringstream objUri;

    if(obj.find("http://") == std::string::npos)
      objUri << "shop:'" << obj << "'";
    else
      objUri << "'" << obj << "'";

    plQuery << "owl_class_properties(" << objUri.str() << ",shop:depthOfProduct," <<
            "literal(type(_,D_XSD))),atom_number(D_XSD,D),"
            << "owl_class_properties(" << objUri.str() << ",shop:widthOfProduct," <<
            "literal(type(_,W_XSD))),atom_number(W_XSD,W),"
            << "owl_class_properties(" << objUri.str() << ",shop:heightOfProduct," <<
            "literal(type(_,H_XSD))),atom_number(H_XSD,H),!.";

    json_prolog::Prolog pl;
    outInfo("Asking query: " << plQuery.str());
    try
    {
      json_prolog::PrologQueryProxy bdgs = pl.query(plQuery.str());
      if(bdgs.begin() == bdgs.end())
      {
        outWarn("No solution to query: " << plQuery.str());
        return false;
      }
      for(auto bdg : bdgs)
      {
        depth = bdg["D"];
        height = bdg["H"];
        width = bdg["W"];
        return true;
        break;
      }
    }
    catch(std::exception e)
    {
      return false;
    }
    return false;
  }

  void filterCloud(const tf::Stamped<tf::Pose> &poseStamped,
                   const double &width, const double &depth, std::string shelf_type)
  {
    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    float minX, minY, minZ;
    float maxX, maxY, maxZ;

    if(shelf_type == "hanging")
    {
      minX = poseStamped.getOrigin().x() - width / 2;
      maxX = poseStamped.getOrigin().x() + width / 2;

      minY = poseStamped.getOrigin().y();
      maxY = poseStamped.getOrigin().y() + 0.3;

      maxZ = poseStamped.getOrigin().z() * 0.99;
      minZ = poseStamped.getOrigin().z() - depth * 0.95;
    }
    else if(shelf_type == "standing")
    {
      minX = poseStamped.getOrigin().x();//+0.02 ; //this is weird
      maxX = minX + width - 0.02;

      minY = poseStamped.getOrigin().y() ;
      maxY = poseStamped.getOrigin().y() + 0.4; //this can vary between 0.3 and 0.5;

      minZ = poseStamped.getOrigin().z() + 0.015  ; //raise with 1 cm
      maxZ = minZ + depth + 0.02 ; //make sure to get point from the top
    }

    pass.setInputCloud(cloudFiltered_);
    pass.setKeepOrganized(true);
    pass.setFilterFieldName("x");// widht of facing
    pass.setFilterLimits(minX, maxX);
    pass.filter(*cloudFiltered_);

    pass.setInputCloud(cloudFiltered_);
    pass.setFilterFieldName("y");//
    pass.setFilterLimits(minY, maxY);
    pass.filter(*cloudFiltered_);

    pass.setInputCloud(cloudFiltered_);
    pass.setFilterFieldName("z");//
    pass.setFilterLimits(minZ, maxZ);
    pass.filter(*cloudFiltered_);

    //    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
    //    sor.setInputCloud(cloudFiltered_);
    //    sor.setMeanK(25);
    //    sor.setStddevMulThresh(3.0);
    //    sor.setKeepOrganized(true);
    //    sor.filter(*cloudFiltered_);
    outInfo("Size of cloud after filtering: " << cloudFiltered_->size());
  }

  void clusterCloud(const double &obj_depth, const pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals)
  {
    pcl::PointCloud<pcl::Label>::Ptr input_labels(new pcl::PointCloud<pcl::Label>);
    pcl::Label label;
    label.label = 0;

    std::vector<bool> ignore_labels;
    ignore_labels.resize(1);
    ignore_labels[0] = false;

    input_labels->height = cloudFiltered_->height;
    input_labels->width = cloudFiltered_->width;
    input_labels->points.resize(cloudFiltered_->points.size(), label);


    pcl ::PointCloud<pcl::Label>::Ptr output_labels(new pcl::PointCloud<pcl::Label>);
    pcl::EuclideanClusterComparator<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>::Ptr ecc(new pcl::EuclideanClusterComparator<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>());
    ecc->setInputCloud(cloudFiltered_);
    ecc->setLabels(input_labels);
    ecc->setExcludeLabels(ignore_labels);
    ecc->setDistanceThreshold(0.06, true);
    ecc->setInputNormals(cloud_normals);
    std::vector<pcl::PointIndices> cluster_i;
    pcl::OrganizedConnectedComponentSegmentation<pcl::PointXYZRGBA, pcl::Label> segmenter(ecc);
    segmenter.setInputCloud(cloudFiltered_);
    segmenter.segment(*output_labels, cluster_i);

    outInfo("Cluster Size before filtering:" << cluster_i.size());

    for(std::vector<pcl::PointIndices>::iterator it = cluster_i.begin();
        it != cluster_i.end();)
    {
      if(it->indices.size() < 300)
        it = cluster_i.erase(it);
      else
        ++it;
    }

    //if two clusters in the same y range
    for(std::vector<pcl::PointIndices>::iterator it1 = cluster_i.begin();
        it1 != cluster_i.end();)
    {
      for(std::vector<pcl::PointIndices>::iterator it2 = cluster_i.begin();
          it2 != cluster_i.end();)
      {
        if(it1 == it2)
        {
          it2++;
          continue;
        }
        Eigen::Vector4f c1, c2;
        pcl::compute3DCentroid(*cloudFiltered_, *it1, c1);
        pcl::compute3DCentroid(*cloudFiltered_, *it2, c2);
        if(std::abs(c1[1] - c2[1]) < obj_depth)
        {
          for(auto idx : it2->indices)
            it1->indices.push_back(idx);
          it2 = cluster_i.erase(it2);
        }
        else it2++;
      }
      it1++;
    }


    outInfo("Found " << cluster_i.size() << " good clusters!");

    float gminX = std::numeric_limits<float>::max(),
          gminZ = std::numeric_limits<float>::max(),
          gmaxX = std::numeric_limits<float>::min(),
          gmaxZ = std::numeric_limits<float>::min();
    for(int i = 0; i < cluster_i.size(); ++i)
    {
      Eigen::Vector4f  min, max;
      pcl::getMinMax3D(*cloudFiltered_, cluster_i[i].indices, min, max);
      float pdepth = std::abs(min[1] - max[1]);
      int count = round(pdepth / obj_depth);

      BoundingBox bb;
      bb.maxPt.x = max[0];
      bb.maxPt.z = max[2];
      bb.minPt.x = min[0];
      bb.minPt.z = min[2];

      if(bb.maxPt.x > gmaxX) gmaxX = bb.maxPt.x;
      if(bb.maxPt.z > gmaxZ) gmaxZ = bb.maxPt.z;
      if(bb.minPt.x < gminX) gminX = bb.minPt.x;
      if(bb.minPt.z < gminZ) gminZ = bb.minPt.z;

      if(count <= 1)
      {
        bb.maxPt.y = max[1];
        bb.minPt.y = min[1];
        cluster_boxes.push_back(bb);
        cluster_indices_.push_back(cluster_i[i]);
      }
      else
      {
        float step = pdepth / count;
        outError("Split this cloud into" << count << " piceses");
        for(int j = 0; j < count; ++j)
        {
          pcl::PointIndices newIndices;
          float minY = min[1] + j * step;
          float maxY = min[1] + (j + 1) * step;
          bb.minPt.y = minY;
          bb.maxPt.y = maxY;
          cluster_boxes.push_back(bb);
          pcl::PassThrough<pcl::PointXYZRGBA> pass;
          pass.setInputCloud(cloudFiltered_);
          pass.setIndices(boost::make_shared<pcl::PointIndices>(cluster_i[i]));
          pass.setFilterFieldName("y");//
          pass.setFilterLimits(minY, maxY); //full depth of four layered shelf
          pass.filter(newIndices.indices);
          if(newIndices.indices.size() > 100) //nois level?
            cluster_indices_.push_back(newIndices);
        }
      }
      outError("THIS: " << count);
    }

    //overwrite all dimensions with biggest BB;
    for(auto &bb : cluster_boxes)
    {
      bb.maxPt.x = gmaxX;
      bb.maxPt.z = gmaxZ;
      bb.minPt.x = gminX;
      bb.minPt.z = gminZ;
    }
  }

  void addToCas(CAS &tcas, std::string objToCount)
  {
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();
    for(int i = 0; i < cluster_indices_.size(); ++i)
    {
      rs::Cluster hyp = rs::create<rs::Cluster>(tcas);
      rs::Detection detection = rs::create<rs::Detection>(tcas);
      detection.source.set("ProductCounter");
      detection.name.set(objToCount);

      //      tf::Stamped<tf::Pose> pose;
      //      pose.setOrigin(tf::Vector3(cluster_boxes[i].ptMax.x));
      //      pose.setRotation(tf::Quaternion(0, 0, 0, 1));
      //      pose.frame_id_ = "map";
      //      uint64_t ts = scene.timestamp();
      //      pose.stamp_ = ros::Time().fromNSec(ts);
      //      rs::PoseAnnotation poseAnnotation  = rs::create<rs::PoseAnnotation>(tcas);
      //      poseAnnotation.source.set("ShelfDetector");
      //      poseAnnotation.world.set(rs::conversion::to(tcas, pose));
      //      poseAnnotation.camera.set(rs::conversion::to(tcas, pose));
      hyp.annotations.append(detection);
      //      hyp.annotations.append(poseAnnotation);
      scene.identifiables.append(hyp);
    }
  }

  bool countObject(CAS &tcas)
  {
    rs::SceneCas cas(tcas);

    std::string objToScan = "" ;
    std::string shelfType = "";
    float distToNextSep = 0.0f;
    tf::Stamped<tf::Pose> poseStamped;
    if(!handleQuery(tcas, objToScan, poseStamped, shelfType, distToNextSep)) return false;
    outInfo("Obj To Scan is: " << objToScan);
    outInfo("Separator location is: [" << poseStamped.getOrigin().x() << "," << poseStamped.getOrigin().y() << "," << poseStamped.getOrigin().z() << "]");
    double height = 0.0, width = 0.0, depth = 0.0;
    getObjectDims(objToScan, height, width, depth);
    outInfo("height = " << height << " width = " << width << " depth  = " << depth);


    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    cas.get(VIEW_CLOUD, *cloud_ptr_);
    cas.get(VIEW_NORMALS, *cloud_normals);
    cas.get(VIEW_COLOR_IMAGE, rgb_);


    cas.get(VIEW_CAMERA_INFO, camInfo_);

    rs::Scene scene = cas.getScene();

    camToWorld_.setIdentity();
    if(scene.viewPoint.has() && !useLocalFrame_)
      rs::conversion::from(scene.viewPoint.get(), camToWorld_);
    else if(useLocalFrame_)
    {
      try
      {

        listener->waitForTransform(localFrameName_, camInfo_.header.frame_id, ros::Time(0)/*camInfo.header.stamp*/, ros::Duration(2));
        listener->lookupTransform(localFrameName_, camInfo_.header.frame_id, ros::Time(0)/*camInfo.header.stamp*/, camToWorld_);
        if(poseStamped.frame_id_ != localFrameName_)
        {

          listener->transformPose(localFrameName_, poseStamped, poseStamped);
          outInfo("New Separator location is: [" << poseStamped.getOrigin().x() << "," << poseStamped.getOrigin().y() << "," << poseStamped.getOrigin().z() << "]");
        }
      }
      catch(tf::TransformException &ex)
      {
        outError(ex.what());
        return UIMA_ERR_NONE;
      }
    }

    Eigen::Affine3d eigenTransform;
    tf::transformTFToEigen(camToWorld_, eigenTransform);
    //        pcl::transformPointCloud<pcl::PointXYZRGBA>(*cloud_ptr_, *cloudFiltered_, eigenTransform);
    pcl::transformPointCloud<pcl::PointXYZRGBA>(*cloud_ptr_, *cloud_ptr_, eigenTransform);
    *cloudFiltered_ = *cloud_ptr_;
    pcl::io::savePCDFileASCII("filtered_cloud.pcd", *cloudFiltered_);
    //0.4 is shelf_depth
    if(width != 0.0 && distToNextSep != 0.0)
      filterCloud(poseStamped, distToNextSep, height, shelfType); //depth of a shelf is given by the shelf_type
    else if(distToNextSep != 0.0)
    {
      ///0.22 m is the biggest height of object we consider if there is no info
      filterCloud(poseStamped, distToNextSep, 0.22, shelfType);
    }
    else
      return false;
    //cluster the filtered cloud and split clusters in chunks of height (on y axes)

    clusterCloud(depth, cloud_normals);
    addToCas(tcas, objToScan);
    return true;
  }
  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    rs::StopWatch clock;


    if(external_)
    {
      countWithExternalAlgo(tcas);
    }
    else
    {
      cloudFiltered_->clear();
      cluster_indices_.clear();
      cluster_boxes.clear();
      countObject(tcas);
      drawOnImage();
    }
    return UIMA_ERR_NONE;
  }

  void drawOnImage()
  {
    for(int j = 0; j < cluster_indices_.size(); ++j)
    {
      for(int i = 0; i < cluster_indices_[j].indices.size(); ++i)
      {
        int index = cluster_indices_[j].indices[i];
        //        int c = cluster_indices[j].indices[i]%disp.cols();
        rgb_.at<cv::Vec3b>(index) = rs::common::cvVec3bColors[j % rs::common::numberOfColors];
      }
    }

    cv_bridge::CvImage outImgMsgs;
    outImgMsgs.header = camInfo_.header;
    outImgMsgs.encoding = sensor_msgs::image_encodings::BGR8;
    outImgMsgs.image = rgb_;
    image_pub_.publish(outImgMsgs.toImageMsg());
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    disp = rgb_.clone();
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    const std::string &cloudname = "cloud";
    double pointSize = 1.0;
    if(firstRun)
    {
      visualizer.addPointCloud(cloudFiltered_, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    else
    {
      visualizer.updatePointCloud(cloudFiltered_,  cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      visualizer.removeAllShapes();
    }
    int idx = 0;


    for(auto &bb : cluster_boxes)
    {
      visualizer.addCube(bb.minPt.x, bb.maxPt.x, bb.minPt.y, bb.maxPt.y, bb.minPt.z, bb.maxPt.z, 1.0, 1.0, 1.0,
                         "box_" + std::to_string(idx));
      idx++;
    }
    visualizer.setRepresentationToWireframeForAllActors();
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(ProductCounter)
