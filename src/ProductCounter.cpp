#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>

//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/utils/common.h>
#include <rs/types/all_types.h>
#include <rs/DrawingAnnotator.h>

#include <rs_refills/common_structs.h>
#include <rs_refills/types/all_types.h>

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
#include <refills_msgs/SeparatorArray.h>
#include <refills_msgs/CountProductsAction.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>

using namespace uima;


class ProductCounter : public DrawingAnnotator
{
private:


  bool external_, saveImgFiles_;
  tf::StampedTransform camToWorld_;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudFiltered_;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr_, cloud_transformed_;
  std::vector<pcl::PointIndices> cluster_indices_;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr separatorPoints_;

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

  std::mutex mtx;

  rs::Facing facing_;

  std::string folderPath_;
public:
  ProductCounter(): DrawingAnnotator(__func__), saveImgFiles_(false), localFrameName_(""), nodeHandle_("~"), it_(nodeHandle_)
  {
    cloudFiltered_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
    cloud_transformed_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
    cloud_ptr_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
    separatorPoints_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
    listener = new tf::TransformListener(nodeHandle_, ros::Duration(10.0));

    image_pub_ = it_.advertise("counting_image", 1, true);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    ctx.extractValue("external", external_);
    ctx.extractValue("saveImgFiles", saveImgFiles_);
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  bool parseQuery(CAS &tcas)// tf::Stamped<tf::Pose> &pose, std::string &shelf_type, float &facingWidth)
  {
    MEASURE_TIME;
    rs::SceneCas cas(tcas);
    rs::Query query = rs::create<rs::Query>(tcas);
    if(!cas.getFS("QUERY", query)) return false;

    std::string queryAsString = query.query();
    if(queryAsString == "")  return false;

    rapidjson::Document doc;
    doc.Parse(queryAsString.c_str());
    if(doc.HasMember("detect"))
    {
      rapidjson::Value &dQuery = doc["detect"];
      if(!dQuery.HasMember("location")) return false;
      localFrameName_ = dQuery["location"].GetString();
    }
    else
      return false;

    return true;
  }

  bool getProductInformation(rs_refills::ProductFacing productFacing, rs::Facing &facing)
  {
    //owl_instance_from_class(shop:'ProductWithAN377954',I),object_dimensions(I,D,W,H).
    MEASURE_TIME;
    std::stringstream plQuery;
    json_prolog::Prolog pl;

    facing.facingId =  productFacing.facingId.get();
    facing.width = static_cast<double>(productFacing.width.get());
    facing.height = static_cast<double>(productFacing.height());
    rs::conversion::from(productFacing.leftSeparatorPose(), facing.leftSeparator);
    rs::conversion::from(productFacing.rightSeparatorPose(), facing.rightSeparator);
    facing.shelfType = static_cast<rs::Facing::ShelfType>(rs::enumFromString(productFacing.shelfType()));
    facing.productId = productFacing.productId.get();

    try
    {
      outInfo("Asking query: " << plQuery.str());

      //get dimenstions and product type of facing
      plQuery.str(std::string());
      plQuery << "owl_class_properties('" << facing.productId << "',shop:articleNumberOfProduct,AN).";

      outInfo("Asking query: " << plQuery.str());
      json_prolog::PrologQueryProxy bdgs = pl.query(plQuery.str());
      if(bdgs.begin() == bdgs.end())
      {
        outError("Product: " << facing.productId << " has no width, height, or product type defined");
        return false;
      }
      for(auto bdg : bdgs)
      {
        facing.gtin = bdg["AN"].toString();
        size_t loc = facing.gtin.find_last_of("GTIN_");
        loc != std::string::npos ? facing.gtin = facing.gtin.substr(loc + 1, facing.gtin.size() - loc - 2) : facing.gtin = "";
      }

      //get the dimenstions of the product on the facing
      plQuery.str(std::string());
      plQuery << "owl_class_properties(" << facing.productId << ",shop:depthOfProduct," <<
              "literal(type(_,D_XSD))),atom_number(D_XSD,D),"
              << "owl_class_properties(" << facing.productId << ",shop:widthOfProduct," <<
              "literal(type(_,W_XSD))),atom_number(W_XSD,W),"
              << "owl_class_properties(" << facing.productId << ",shop:heightOfProduct," <<
              "literal(type(_,H_XSD))),atom_number(H_XSD,H),!.";
      outInfo("Asking query: " << plQuery.str());
      bdgs = pl.query(plQuery.str());
      if(bdgs.begin() == bdgs.end())
      {
        outWarn("No solution to query: " << plQuery.str());
        facing.productDims.d = 0.41;
        facing.productDims.h = facing.height;
        facing.productDims.w = facing.width;
      }
      for(auto bdg : bdgs)
      {
        facing.productDims.d = bdg["D"];
        facing.productDims.h = bdg["H"];
        facing.productDims.w = bdg["W"];
        break;
      }
    }
    catch(tf::TransformException &ex)
    {
      outError("Exception: " << ex.what());
      return false;
    }
    catch(std::exception e)
    {
      outError("Exception when looking up facing information: " << e.what());
      return false;
    }
    return true;
  }

  void filterCloud(rs::Facing facing)
  {
    MEASURE_TIME;
    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    float minX, minY, minZ;
    float maxX, maxY, maxZ;

    if(facing.shelfType == rs::Facing::ShelfType::HANGING)
    {
      minX = facing.leftSeparator.getOrigin().x() - facing.width / 2;
      maxX = facing.leftSeparator.getOrigin().x() + facing.width / 2;

      minY = facing.leftSeparator.getOrigin().y() - 0.04;
      maxY = facing.leftSeparator.getOrigin().y() + 0.3;

      maxZ = facing.leftSeparator.getOrigin().z();
      minZ = facing.leftSeparator.getOrigin().z() - facing.height;
    }
    else if(facing.shelfType == rs::Facing::ShelfType::STANDING)
    {
      float xOffset = 0.01;
      if(fabs(facing.leftSeparator.getOrigin().x() - facing.rightSeparator.getOrigin().x()) < 0.025)
        xOffset = 0.00;

      //xOffste= (poseStamped.getOrigin().x()- pose2Stamped.getOrigin().x())
      minX = facing.leftSeparator.getOrigin().x() + xOffset;
      maxX = facing.rightSeparator.getOrigin().x() - xOffset;

      minY = facing.leftSeparator.getOrigin().y() - 0.04; //move closer to cam with 2 cm
      maxY = minY + 0.41; //this can vary between 0.3 and 0.5;

      minZ = facing.leftSeparator.getOrigin().z() + 0.01; //raise with 2.5 cm
      maxZ = facing.leftSeparator.getOrigin().z() + facing.height;
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
    outInfo("Size of cloud after filtering: " << cloudFiltered_->size());
  }

  int clusterCloud(const double &obj_depth, const pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals)
  {
    MEASURE_TIME;
    pcl::PointCloud<pcl::Label>::Ptr input_labels(new pcl::PointCloud<pcl::Label>);
    pcl::Label label;
    label.label = 0;

    std::vector<bool> ignore_labels;
    ignore_labels.resize(1);
    ignore_labels[0] = false;

    input_labels->height = cloudFiltered_->height;
    input_labels->width = cloudFiltered_->width;
    input_labels->points.resize(cloudFiltered_->points.size(), label);

    pcl::PointCloud<pcl::Label>::Ptr output_labels(new pcl::PointCloud<pcl::Label>);
    pcl::EuclideanClusterComparator<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>::Ptr ecc(new pcl::EuclideanClusterComparator<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>());
    ecc->setInputCloud(cloudFiltered_);
    ecc->setLabels(input_labels);
    ecc->setExcludeLabels(ignore_labels);
    ecc->setDistanceThreshold(0.06, false);///Why is this false??? if true clustering is sometimtes weird...
    ecc->setInputNormals(cloud_normals);
    std::vector<pcl::PointIndices> cluster_i;
    pcl::OrganizedConnectedComponentSegmentation<pcl::PointXYZRGBA, pcl::Label> segmenter(ecc);
    segmenter.setInputCloud(cloudFiltered_);
    segmenter.segment(*output_labels, cluster_i);

    outInfo("Cluster Size before filtering:" << cluster_i.size());

    for(std::vector<pcl::PointIndices>::iterator it = cluster_i.begin();
        it != cluster_i.end();)
    {
      if(it->indices.size() < 400)
        it = cluster_i.erase(it);
      else
        ++it;
    }
    outInfo("Cluster Size after filtering:" << cluster_i.size());

    //if two clusters in the same y range
    std::vector<pcl::PointIndices> mergedClusterIndices;

    for(int i = 0; i < cluster_i.size(); ++i)
    {
      Eigen::Vector4f c1;
      pcl::compute3DCentroid(*cloudFiltered_, cluster_i[i], c1);
      bool merged = false;
      for(int j = 0; j < mergedClusterIndices.size(); j++)
      {
        Eigen::Vector4f c2;
        pcl::compute3DCentroid(*cloudFiltered_, mergedClusterIndices[j], c2);
        if(std::abs(c1[1] - c2[1]) < obj_depth)
        {
          mergedClusterIndices[j].indices.insert(mergedClusterIndices[j].indices.end(),
                                                 cluster_i[i].indices.begin(),
                                                 cluster_i[i].indices.end());
          merged = true;
          break;
        }
      }
      if(!merged)
        mergedClusterIndices.push_back(cluster_i[i]);
    }
    outInfo("Found " << mergedClusterIndices.size() << " good clusters after filtering and merging!");

    float gminX = std::numeric_limits<float>::max(),
          gminZ = std::numeric_limits<float>::max(),
          gmaxX = std::numeric_limits<float>::min(),
          gmaxZ = std::numeric_limits<float>::min();
    for(int i = 0; i < mergedClusterIndices.size(); ++i)
    {
      Eigen::Vector4f  min, max;
      pcl::getMinMax3D(*cloudFiltered_, mergedClusterIndices[i].indices, min, max);
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
        cluster_indices_.push_back(mergedClusterIndices[i]);
      }
      else
      {
        float step = pdepth / count;
        outError("Split this cloud into " << count << " piceses");
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
          pass.setIndices(boost::make_shared<pcl::PointIndices>(mergedClusterIndices[i]));
          pass.setFilterFieldName("y");//
          pass.setFilterLimits(minY, maxY); //full depth of four layered shelf
          pass.filter(newIndices.indices);
          if(newIndices.indices.size() > 100) //nois level?
            cluster_indices_.push_back(newIndices);
        }
      }
    }

    //overwrite all dimensions with biggest BB;
    for(auto &bb : cluster_boxes)
    {
      bb.maxPt.x = gmaxX;
      bb.maxPt.z = gmaxZ;
      bb.minPt.x = gminX;
      bb.minPt.z = gminZ;
    }
    return cluster_indices_.size();
  }

  bool countObject(CAS &tcas)
  {
    rs::SceneCas cas(tcas);
    if(!parseQuery(tcas)) return false;
    rs::Scene scene = cas.getScene();
    std::vector<rs_refills::ProductFacing> facings;
    scene.identifiables.filter(facings);

    for(rs_refills::ProductFacing &product_facing : facings)
    {
      rs::Facing facing_;
      cv::Rect facing_roi;
      rs::conversion::from(product_facing.rois().roi(),facing_roi);
      cv::rectangle(rgb_, facing_roi, cv::Scalar(0, 255, 0));

      if(getProductInformation(product_facing,facing_))
      {
        std::lock_guard<std::mutex> lock(mtx);
        outInfo("Facing To Scan is: " << facing_.facingId);
        outInfo("Separator location is: [" << facing_.leftSeparator.getOrigin().x() << ","
                << facing_.leftSeparator.getOrigin().y() << ","
                << facing_.leftSeparator.getOrigin().z() << "]");

        //      getObjectDims(objToScan, height, width, depth);
        outInfo("height = " << facing_.productDims.h << " width = " << facing_.productDims.w << " depth  = " << facing_.productDims.d);

        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        cas.get(VIEW_CLOUD, *cloud_ptr_);
        cas.get(VIEW_NORMALS, *cloud_normals);
        cas.get(VIEW_COLOR_IMAGE, rgb_);
        cas.get(VIEW_CAMERA_INFO, camInfo_);

        try
        {
          listener->waitForTransform(localFrameName_, camInfo_.header.frame_id, ros::Time(0), ros::Duration(2));
          listener->lookupTransform(localFrameName_, camInfo_.header.frame_id,  ros::Time(0), camToWorld_);
          if(facing_.leftSeparator.frame_id_ != localFrameName_)
          {
            listener->transformPose(localFrameName_, facing_.leftSeparator, facing_.leftSeparator);
            outInfo("New Separator location is: [" << facing_.leftSeparator.getOrigin().x() << "," << facing_.leftSeparator.getOrigin().y() << "," << facing_.leftSeparator.getOrigin().z() << "]");
          }
          if(facing_.rightSeparator.frame_id_ != localFrameName_ && facing_.shelfType != rs::Facing::ShelfType::HANGING)
          {
            listener->transformPose(localFrameName_, facing_.rightSeparator, facing_.rightSeparator);
            outInfo("New Separator location is: [" << facing_.rightSeparator.getOrigin().x() << ","
                    << facing_.rightSeparator.getOrigin().y() << ","
                    << facing_.rightSeparator.getOrigin().z() << "]");
          }
        }
        catch(tf::TransformException &ex)
        {
          outError(ex.what());
          return UIMA_ERR_NONE;
        }

        Eigen::Affine3d eigenTransform;
        tf::transformTFToEigen(camToWorld_, eigenTransform);
        pcl::transformPointCloud<pcl::PointXYZRGBA>(*cloud_ptr_, *cloudFiltered_, eigenTransform);
        *cloud_transformed_ = * cloudFiltered_;

        filterCloud(facing_);

        int count = clusterCloud(facing_.productDims.d, cloud_normals);
        rs_refills::ProductCount prod_count = rs::create<rs_refills::ProductCount>(tcas);
        prod_count.product_count.set(count);
        product_facing.annotations.append(prod_count);
//        addToCas(tcas, facing_);
      }
    }
    separatorPoints_->clear();
    return true;
  }


  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    MEASURE_TIME;

    cloudFiltered_->clear();
    cluster_indices_.clear();
    cluster_boxes.clear();

    countObject(tcas);

    drawOnImage();

    return UIMA_ERR_NONE;
  }


  void drawOnImage()
  {
    for(int j = 0; j < cluster_indices_.size(); ++j)
    {
      for(int i = 0; i < cluster_indices_[j].indices.size(); ++i)
      {
        int index = cluster_indices_[j].indices[i];
        rgb_.at<cv::Vec3b>(index) = rs::common::cvVec3bColors[j % rs::common::numberOfColors];
      }
    }

    //TOOD: use image transport
    cv_bridge::CvImage outImgMsgs;
    outImgMsgs.header = camInfo_.header;
    outImgMsgs.encoding = sensor_msgs::image_encodings::BGR8;
    outImgMsgs.image = rgb_;
    image_pub_.publish(outImgMsgs.toImageMsg());
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    if(!rgb_.empty())
      disp = rgb_.clone();
    else disp = cv::Mat::ones(480, 640, CV_8UC3);
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    const std::string &cloudname = "cloud_filtered";
    const std::string &cloudTransformedname = "cloud_original";
    double pointSize = 1.0;
    if(firstRun)
    {
      visualizer.addPointCloud(cloudFiltered_, cloudname);
      visualizer.addPointCloud(cloud_transformed_, cloudTransformedname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize / 2.0, cloudTransformedname);
    }
    else
    {
      visualizer.updatePointCloud(cloudFiltered_,  cloudname);
      visualizer.updatePointCloud(cloud_transformed_, cloudTransformedname);
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
