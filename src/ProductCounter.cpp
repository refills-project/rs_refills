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
  bool external_;
  tf::StampedTransform camToWorld;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudFiltered_;
  std::vector<pcl::PointIndices> cluster_indices_;

  cv::Mat rgb_;

  struct BoundingBox
  {
    pcl::PointXYZ minPt, maxPt;
  };

  std::vector<BoundingBox> cluster_boxes;
public:

  ProductCounter(): DrawingAnnotator(__func__)
  {
    cloudFiltered_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
  }
  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    ctx.extractValue("external", external_);
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

  bool handleQuery(CAS &tcas, std::string &obj, tf::Stamped<tf::Pose> &pose)
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
          if(doc["detect"].HasMember("type"))
          {
            obj = doc["detect"]["type"].GetString();
          }
          if(doc["detect"].HasMember("pose"))
          {
            tf::Vector3 position;
            position.setX(doc["detect"]["pose"]["position"]["x"].GetFloat());
            position.setY(doc["detect"]["pose"]["position"]["y"].GetFloat());
            position.setZ(doc["detect"]["pose"]["position"]["z"].GetFloat());
            pose.frame_id_ = doc["detect"]["pose"]["frame_id"].GetString();
            pose.setOrigin(position);
            pose.setRotation(tf::Quaternion(0, 0, 0, 1));
            pose.stamp_ = ros::Time::now();
          }
          else
            return false;
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
    plQuery << "owl_class_properties(shop:'" << obj << "',shop:depthOfProduct," <<
            "literal(type(_,D_XSD))),atom_number(D_XSD,D),"
            << "owl_class_properties(shop:'" << obj << "',shop:widthOfProduct," <<
            "literal(type(_,W_XSD))),atom_number(W_XSD,W),"
            << "owl_class_properties(shop:'" << obj << "',shop:heightOfProduct," <<
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
                   const double &width, const double &height, const double &depth)
  {
    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    pass.setInputCloud(cloudFiltered_);
    pass.setKeepOrganized(true);
    pass.setFilterFieldName("x");// widht of facing
    pass.setFilterLimits(poseStamped.getOrigin().x() * 1.02, poseStamped.getOrigin().x() + width);
    pass.filter(*cloudFiltered_);

    pass.setInputCloud(cloudFiltered_);
    pass.setFilterFieldName("y");//
    pass.setFilterLimits(poseStamped.getOrigin().y(), poseStamped.getOrigin().y() + 0.4); //full depth of four layered shelf
    pass.filter(*cloudFiltered_);

    pass.setInputCloud(cloudFiltered_);
    pass.setFilterFieldName("z");//
    pass.setFilterLimits(poseStamped.getOrigin().z() * 1.01, poseStamped.getOrigin().z() + depth * 1.05);
    pass.filter(*cloudFiltered_);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
    sor.setInputCloud(cloudFiltered_);
    sor.setMeanK(25);
    sor.setStddevMulThresh(2.5);
    sor.setKeepOrganized(true);
    sor.filter(*cloudFiltered_);
    outInfo("Size of cloud after filtering: " << cloudFiltered_->size());
  }

  void clusterCloud(const double &height, const pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals)
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
      if(it->indices.size() < 500)
        it = cluster_i.erase(it);
      else
        ++it;
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
      int count = round(pdepth / height);

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
          cluster_indices_.push_back(newIndices);
        }
      }
      outError("THIS: " << count);
    }

    //overwrite all dimensions with biggest BB;
    for (auto &bb :cluster_boxes)
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
      uint64_t ts = scene.timestamp();
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
    tf::Stamped<tf::Pose> poseStamped;
    if(!handleQuery(tcas, objToScan, poseStamped)) return false;
    outInfo("Obj To Scan is: " << objToScan);
    outInfo("Separator location is: [" << poseStamped.getOrigin().x() << "," << poseStamped.getOrigin().y() << "," << poseStamped.getOrigin().z() << "]");
    double height, width, depth;
    if(!getObjectDims(objToScan, height, width, depth)) return false;
    outInfo("height = " << height << " width = " << width << " depth  = " << depth);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    cas.get(VIEW_CLOUD, *cloud_ptr);
    cas.get(VIEW_NORMALS, *cloud_normals);
    cas.get(VIEW_COLOR_IMAGE, rgb_);
    rs::Scene scene = cas.getScene();
    camToWorld.setIdentity();
    if(scene.viewPoint.has())
      rs::conversion::from(scene.viewPoint.get(), camToWorld);

    Eigen::Affine3d eigenTransform;
    tf::transformTFToEigen(camToWorld, eigenTransform);
    pcl::transformPointCloud<pcl::PointXYZRGBA>(*cloud_ptr, *cloudFiltered_, eigenTransform);

    filterCloud(poseStamped, width, height, depth);

    //cluster the filtered cloud and split clusters in chunks of height (on y axes)
    clusterCloud(height, cloud_normals);
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
    }
    return UIMA_ERR_NONE;
  }


  void drawImageWithLock(cv::Mat &disp)
  {
    disp = rgb_.clone();
    for(int j = 0; j < cluster_indices_.size(); ++j)
    {
      for(int i = 0; i < cluster_indices_[j].indices.size(); ++i)
      {
        int index = cluster_indices_[j].indices[i];
        //        int c = cluster_indices[j].indices[i]%disp.cols();
        disp.at<cv::Vec3b>(index) = rs::common::cvVec3bColors[j % rs::common::numberOfColors];
      }
    }
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
      visualizer.updatePointCloud(cloudFiltered_, cloudname);
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
