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

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>

using namespace uima;


class ProductCounter : public DrawingAnnotator
{
private:

  struct Facing {

    struct obj_dims {
      double w, h, d;
    } productDims;

    double width;
    double height;
    enum class ShelfType {HANGING, STANDING};
    ShelfType shelfType;

    tf::Stamped<tf::Pose> leftSeparator, rightSeparator, topRightCorner;
    std::string gtin;
    std::string dan;
    std::string productId;
    std::string facingId;

    cv::Rect rect, rect_hires_;
    cv::Mat mask;
  };


  bool external_, useLocalFrame_, saveImgFiles_;
  tf::StampedTransform camToWorld_;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudFiltered_;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr_, cloud_transformed_;
  std::vector<pcl::PointIndices> cluster_indices_;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr separatorPoints_;

  cv::Mat rgb_;
  std::string localFrameName_;

  struct BoundingBox {
    pcl::PointXYZ minPt, maxPt;
  };


  tf::TransformListener *listener;
  ros::Subscriber separatorSubscriber_;

  std::vector<BoundingBox> cluster_boxes;

  ros::NodeHandle nodeHandle_;

//  image_transport::Publisher image_pub_;
//  image_transport::ImageTransport it_;

  tf::Stamped<tf::Pose> separatorPoseInImage_, nextSeparatorPoseInImage_, originalSeparator1PoseImageFrame_, topRightCornerInImage_, originalSeparator2PoseImageFrame_,
                    leftSeparatorPoseInFlirImage_, rightSeparatorPoseInFlirImage_, topRightCornerInFlirImage_;

  sensor_msgs::CameraInfo camInfo_, flir_cam_info_;

  std::mutex mtx;

  Facing facing_;

  std::string folderPath_;
public:
  ProductCounter(): DrawingAnnotator(__func__), useLocalFrame_(false), saveImgFiles_(false), localFrameName_(""), nodeHandle_("~")/*, it_(nodeHandle_)*/
  {
    cloudFiltered_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
    cloud_transformed_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
    cloud_ptr_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
    separatorPoints_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
    listener = new tf::TransformListener(nodeHandle_, ros::Duration(10.0));

//    image_pub_ = it_.advertise("counting_image", 1, true);

    separatorSubscriber_ = nodeHandle_.subscribe("/separator_marker_detector_node/data_out", 50, &ProductCounter::separatorCb, this);

//    //create a new folder for saving images into
//    std::string packagePath = ros::package::getPath("rs_refills") + "/data_out";
//    boost::posix_time::ptime posixTime = ros::Time::now().toBoost();
//    std::string iso_time_str = boost::posix_time::to_iso_extended_string(posixTime);
//    folderPath_ = packagePath + "/" + iso_time_str;
//    outWarn(folderPath_);
//    boost::filesystem::path path(folderPath_);
//    if(!boost::filesystem::exists(path)) {
//      outInfo("Creating folder: " << path.string());
//      boost::filesystem::create_directory(path);
//    }
//    else {
//      outWarn("How can this already exist?");
//    }
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    ctx.extractValue("external", external_);
    ctx.extractValue("use_local_frame", useLocalFrame_);
    ctx.extractValue("saveImgFiles", saveImgFiles_);



    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  void separatorCb(const refills_msgs::SeparatorArrayPtr &msg)
  {
    std::lock_guard<std::mutex> lock(mtx);
    if(localFrameName_ != "") {
      for(auto m : msg->separators) {
        tf::Stamped<tf::Pose> poseStamped, poseBase;
        tf::poseStampedMsgToTF(m.separator_pose, poseStamped);
        try {
          listener->waitForTransform(localFrameName_, poseStamped.frame_id_, poseStamped.stamp_, ros::Duration(1.0));
          listener->transformPose(localFrameName_, poseStamped.stamp_, poseStamped, poseStamped.frame_id_, poseBase);

          pcl::PointXYZRGBA pt;
          pt.x = poseBase.getOrigin().x();
          pt.y = poseBase.getOrigin().y();
          pt.z = poseBase.getOrigin().z();

          separatorPoints_->points.push_back(pt);
        }
        catch(tf::TransformException ex) {
          outWarn(ex.what());
        }
      }
    }
  }

  bool parseQuery(CAS &tcas, std::string &facingID)// tf::Stamped<tf::Pose> &pose, std::string &shelf_type, float &facingWidth)
  {
    MEASURE_TIME;
    rs::SceneCas cas(tcas);
    rs::Query query = rs::create<rs::Query>(tcas);
    if(!cas.getFS("QUERY", query)) return false;

    std::string queryAsString = query.query();
    if(queryAsString == "")  return false;

    rapidjson::Document doc;
    doc.Parse(queryAsString.c_str());
    if(doc.HasMember("detect")) {
      rapidjson::Value &dQuery = doc["detect"];

      if(!dQuery.HasMember("facing")) return false;
      facingID = dQuery["facing"].GetString();

      if(!dQuery.HasMember("location")) return false;
      localFrameName_ = dQuery["location"].GetString();
    }
    else
      return false;

    return true;
  }

  bool getFacingInformation(Facing &facing)
  {
    //owl_instance_from_class(shop:'ProductWithAN377954',I),object_dimensions(I,D,W,H).
    MEASURE_TIME;
    std::stringstream plQuery;
    json_prolog::Prolog pl;
    plQuery << "shelf_facing(F,'" << facing.facingId << "'),shelf_layer_standing(F).";

    try {
      outInfo("Asking query: " << plQuery.str());
      json_prolog::PrologQueryProxy bdgs = pl.query(plQuery.str());
      if(bdgs.begin() == bdgs.end()) {
        outInfo("facing is on a mounting hanging ");
        facing.shelfType =Facing::ShelfType::HANGING;
      }
      else {
        outInfo("Standing shelf");
        facing.shelfType = Facing::ShelfType::STANDING;
      }

      if(facing.shelfType == Facing::ShelfType::STANDING) {
        //get the left and Right separators:
        plQuery.str(std::string(""));
        plQuery << "rdf_has('" << facing.facingId << "', shop:leftSeparator, L), object_perception_affordance_frame_name(L,LFrameName),"
                << "rdf_has('" << facing.facingId << "', shop:rightSeparator,R), object_perception_affordance_frame_name(R,RFrameName).";
        outInfo("Asking query: " << plQuery.str());
        bdgs = pl.query(plQuery.str());
        if(bdgs.begin() == bdgs.end()) {
          outError("No results found the left and right separator. are you sure this is the right facing type?");
          return false;
        }
        std::string leftSepTFId, rightSepTFId;
        for(auto bdg : bdgs) {
          leftSepTFId = bdg["LFrameName"].toString();
          leftSepTFId = leftSepTFId.substr(1, leftSepTFId.size() - 2);
          rightSepTFId = bdg["RFrameName"].toString();
          rightSepTFId = rightSepTFId.substr(1, rightSepTFId.size() - 2);
          break;
        }
        tf::StampedTransform leftSep, rightSep;
        listener->waitForTransform(localFrameName_, leftSepTFId, ros::Time(0), ros::Duration(2.0));
        listener->lookupTransform(localFrameName_, leftSepTFId, ros::Time(0), leftSep);
        listener->waitForTransform(localFrameName_, rightSepTFId, ros::Time(0), ros::Duration(2.0));
        listener->lookupTransform(localFrameName_, rightSepTFId, ros::Time(0), rightSep);
        facing.leftSeparator.setRotation(leftSep.getRotation());
        facing.leftSeparator.setOrigin(leftSep.getOrigin());
        facing.leftSeparator.frame_id_ = leftSep.frame_id_;
        facing.leftSeparator.stamp_ = leftSep.stamp_;

        facing.rightSeparator.setRotation(rightSep.getRotation());
        facing.rightSeparator.setOrigin(rightSep.getOrigin());
        facing.rightSeparator.frame_id_ = rightSep.frame_id_;
        facing.rightSeparator.stamp_ = rightSep.stamp_;
      }
      else if(facing.shelfType == Facing::ShelfType::HANGING) {
        plQuery.str(std::string(""));
        plQuery << "rdf_has('" << facing.facingId << "', shop:mountingBarOfFacing, M), object_perception_affordance_frame_name(M,MFrameName).";
        outInfo("Asking query: " << plQuery.str());
        bdgs = pl.query(plQuery.str());
        if(bdgs.begin() == bdgs.end()) {
          outError("This Facing has no mountint Bar...WTF");
          return false;
        }
        std::string mountingBarTFId;
        for(auto bdg : bdgs) {
          mountingBarTFId = bdg["MFrameName"].toString();
          mountingBarTFId = mountingBarTFId.substr(1, mountingBarTFId.size() - 2);
          break;
        }
        tf::StampedTransform mountingBar;
        listener->lookupTransform(localFrameName_, mountingBarTFId, ros::Time(0), mountingBar);
        facing.leftSeparator.setRotation(mountingBar.getRotation());
        facing.leftSeparator.setOrigin(mountingBar.getOrigin());
        facing.leftSeparator.frame_id_ = mountingBar.frame_id_;
        facing.leftSeparator.stamp_ = mountingBar.stamp_;
      }

      //get dimenstions and product type of facing
      plQuery.str(std::string());
      plQuery << "shelf_facing_product_type('" << facing.facingId << "', P),"
              << "owl_class_properties(P,shop:articleNumberOfProduct,AN),"
 	      << "rdf_has_prolog(AN,shop:dan,DAN),"
              << "comp_facingWidth('" << facing.facingId << "',literal(type(_, W_XSD))),atom_number(W_XSD,W),"
              << "comp_facingHeight('" << facing.facingId << "',literal(type(_, H_XSD))),atom_number(H_XSD,H).";
      outInfo("Asking query: " << plQuery.str());
      bdgs = pl.query(plQuery.str());
      if(bdgs.begin() == bdgs.end()) {
        outError("Facing: " << facing.facingId << " has no width, height, or product type defined");
        return false;
      }
      for(auto bdg : bdgs) {
        facing.width = bdg["W"];
        facing.height = bdg["H"];
        facing.productId = bdg["P"].toString();
        facing.gtin = bdg["AN"].toString();
        facing.dan = bdg["DAN"].toString();
        size_t loc = facing.gtin.find_last_of("GTIN_");
//      size_t loc2 = facing.dan.find_last_of("AN");
        loc != std::string::npos ? facing.gtin = facing.gtin.substr(loc + 1, facing.gtin.size() - loc - 2) : facing.gtin = "";
//        loc2 != std::string::npos ? facing.dan = facing.dan.substr(loc2 + 1, facing.dan.size() - loc2 - 2) : facing.dan = "";
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
      if(bdgs.begin() == bdgs.end()) {
        outWarn("No solution to query: " << plQuery.str());
        facing.productDims.d = 0.41;
        facing.productDims.h = facing.height;
        facing.productDims.w = facing.width;
      }
      for(auto bdg : bdgs) {
        facing.productDims.d = bdg["D"];
        facing.productDims.h = bdg["H"];
        facing.productDims.w = bdg["W"];
        break;
      }
    }
    catch(tf::TransformException &ex) {
      outError("Exception: " << ex.what());
      return false;
    }
    catch(std::exception e) {
      outError("Exception when looking up facing information: " << e.what());
      return false;
    }
    return true;
  }

  void filterCloud(Facing facing)
  {
    MEASURE_TIME;
    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    float minX, minY, minZ;
    float maxX, maxY, maxZ;

    if(facing.shelfType == Facing::ShelfType::HANGING) {
      minX = facing.leftSeparator.getOrigin().x() - facing.width / 2;
      maxX = facing.leftSeparator.getOrigin().x() + facing.width / 2;

      minY = facing.leftSeparator.getOrigin().y() - 0.04;
      maxY = facing.leftSeparator.getOrigin().y() + 0.3;

      maxZ = facing.leftSeparator.getOrigin().z();
      minZ = facing.leftSeparator.getOrigin().z() - facing.height;
    }
    else if(facing.shelfType == Facing::ShelfType::STANDING) {
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
    //    pcl::io::savePCDFile("cloud_filtered.pcd", *cloudFiltered_);
    outInfo("Size of cloud after filtering: " << cloudFiltered_->size());
  }

  void clusterCloud(const double &obj_depth, const pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals)
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
        it != cluster_i.end();) {
      if(it->indices.size() < 400)
        it = cluster_i.erase(it);
      else
        ++it;
    }
    outInfo("Cluster Size after filtering:" << cluster_i.size());

    //if two clusters in the same y range
    std::vector<pcl::PointIndices> mergedClusterIndices;

    for(int i = 0; i < cluster_i.size(); ++i) {
      Eigen::Vector4f c1;
      pcl::compute3DCentroid(*cloudFiltered_, cluster_i[i], c1);
      bool merged = false;
      for(int j = 0; j < mergedClusterIndices.size(); j++) {
        Eigen::Vector4f c2;
        pcl::compute3DCentroid(*cloudFiltered_, mergedClusterIndices[j], c2);
        if(std::abs(c1[1] - c2[1]) < obj_depth) {
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
    for(int i = 0; i < mergedClusterIndices.size(); ++i) {
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

      if(count <= 1) {
        bb.maxPt.y = max[1];
        bb.minPt.y = min[1];
        cluster_boxes.push_back(bb);
        cluster_indices_.push_back(mergedClusterIndices[i]);
      }
      else {
        float step = pdepth / count;
        outError("Split this cloud into " << count << " piceses");
        for(int j = 0; j < count; ++j) {
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
    for(auto &bb : cluster_boxes) {
      bb.maxPt.x = gmaxX;
      bb.maxPt.z = gmaxZ;
      bb.minPt.x = gminX;
      bb.minPt.z = gminZ;
    }
  }

  void  addToCas(CAS &tcas, Facing facing)
  {
    MEASURE_TIME;
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();
    rs::Scene scene_flir = cas.getScene(1);


    rs::ObjectHypothesis facingHyp = rs::create<rs::ObjectHypothesis>(tcas);

    //ACHTUNG!!!:: this is the rect in a diferent camera: flir hight res...yes it's an ugly thing,
    // that is why I am commenting it here!!!!
    cv::Rect rect_hires = facing_.rect_hires_;


    rs::ImageROI roi = rs::create<rs::ImageROI>(tcas);
    roi.roi(rs::conversion::to(tcas, facing_.rect));
    roi.roi_hires(rs::conversion::to(tcas, rect_hires));
    cv::Mat mask = cv::Mat::ones(cv::Size(facing_.rect.size()), CV_8U);
    cv::Mat mask_hires = cv::Mat::ones(cv::Size(rect_hires.size()), CV_8U);
    roi.mask(rs::conversion::to(tcas, mask));
    roi.mask_hires(rs::conversion::to(tcas, mask_hires));

    facingHyp.rois(roi);

    rs::Detection detection = rs::create<rs::Detection>(tcas);
    detection.source.set("FacingDetection");
    detection.name.set(std::string(facing.gtin));

    rs::Classification cl = rs::create<rs::Classification>(tcas);
    cl.featurename.set(std::string(facing.dan));

    facingHyp.annotations.append(detection);
    facingHyp.annotations.append(cl);
    scene.identifiables.append(facingHyp);

//    rs::PoseAnnotation left_pose_annotation = rs::create<rs::PoseAnnotation>(tcas);
//    rs::PoseAnnotation right_pose_annotation = rs::create<rs::PoseAnnotation>(tcas);
//    rs::PoseAnnotation top_right_pose_annotation = rs::create<rs::PoseAnnotation>(tcas);

//    left_pose_annotation.world.set( rs::conversion::to(cas, facing_.leftSeparator));
//    left_pose_annotation.source.set("left");
//    right_pose_annotation.world.set( rs::conversion::to(cas, facing_.rightSeparator));
//    right_pose_annotation.source.set("right");
//    top_right_pose_annotation.world.set( rs::conversion::to(cas, facing_.topRightCorner));
//    top_right_pose_annotation.source.set("top-right");

//    facingHyp.annotations.appen(left_pose_annotation);
//    facingHyp.annotations.appen(right_pose_annotation);
//    facingHyp.annotations.appen(top_right_pose_annotation);

    for(int i = 0; i < cluster_indices_.size(); ++i) {
      rs::ObjectHypothesis hyp = rs::create<rs::ObjectHypothesis>(tcas);
      rs::Detection detection = rs::create<rs::Detection>(tcas);
      detection.source.set("ProductCounter");
      detection.name.set(facing.productId);
      hyp.annotations.append(detection);
      scene.identifiables.append(hyp);
    }
  }

  bool countObject(CAS &tcas)
  {
    rs::SceneCas cas(tcas);

    facing_ = Facing();
    if(!parseQuery(tcas, facing_.facingId)) return false;

    if(getFacingInformation(facing_)) {
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
      cas.get(VIEW_CAMERA_INFO, flir_cam_info_,1);

      try {
        listener->waitForTransform(localFrameName_, camInfo_.header.frame_id, ros::Time(0), ros::Duration(2));
        listener->lookupTransform(localFrameName_, camInfo_.header.frame_id,  ros::Time(0), camToWorld_);
        if(facing_.leftSeparator.frame_id_ != localFrameName_) {
          listener->transformPose(localFrameName_, facing_.leftSeparator, facing_.leftSeparator);
          outInfo("New Separator location is: [" << facing_.leftSeparator.getOrigin().x() << "," << facing_.leftSeparator.getOrigin().y() << "," << facing_.leftSeparator.getOrigin().z() << "]");
        }
        if(facing_.rightSeparator.frame_id_ != localFrameName_ && facing_.shelfType != Facing::ShelfType::HANGING) {
          listener->transformPose(localFrameName_, facing_.rightSeparator, facing_.rightSeparator);
          outInfo("New Separator location is: [" << facing_.rightSeparator.getOrigin().x() << ","
                  << facing_.rightSeparator.getOrigin().y() << ","
                  << facing_.rightSeparator.getOrigin().z() << "]");
        }

        if(facing_.shelfType == facing_.ShelfType::STANDING) {
          //let's find the closest separators in the current detection;
          listener->transformPose(camInfo_.header.frame_id, facing_.leftSeparator, originalSeparator1PoseImageFrame_);
          listener->transformPose(camInfo_.header.frame_id, facing_.rightSeparator, originalSeparator2PoseImageFrame_);

          //start fixing poses of separators
          if(separatorPoints_->size() > 2) {
            outInfo("Using current detection of separators to fix positions");
            pcl::PointXYZRGBA s1, s2;
            s1.x = facing_.leftSeparator.getOrigin().x();
            s1.y = facing_.leftSeparator.getOrigin().y();
            s1.z = facing_.leftSeparator.getOrigin().z();

            s2.x = facing_.rightSeparator.getOrigin().x();
            s2.y = facing_.rightSeparator.getOrigin().y();
            s2.z = facing_.rightSeparator.getOrigin().z();

            pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
            tree->setInputCloud(separatorPoints_);
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            if(tree->radiusSearch(s1, 0.04, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
              outInfo("Found " << pointIdxRadiusSearch.size() << " close to separator 1");
              pcl::PointXYZRGBA np = separatorPoints_->points[pointIdxRadiusSearch[0]];
              facing_.leftSeparator.setOrigin(tf::Vector3(np.x, np.y, np.z));
              separatorPoints_->points.erase(separatorPoints_->points.begin() + pointIdxRadiusSearch[0]);
            }

            tree->setInputCloud(separatorPoints_);
            if(tree->radiusSearch(s2, 0.04, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
              outInfo("Found " << pointIdxRadiusSearch.size() << " close to separator 2");
              pcl::PointXYZRGBA np = separatorPoints_->points[pointIdxRadiusSearch[0]];
              facing_.rightSeparator.setOrigin(tf::Vector3(np.x, np.y, np.z));
            }
          }
          //end fixing positions;
        }

        listener->transformPose(camInfo_.header.frame_id, facing_.leftSeparator, separatorPoseInImage_);
        if(facing_.shelfType == Facing::ShelfType::STANDING)
          listener->transformPose(camInfo_.header.frame_id, facing_.rightSeparator, nextSeparatorPoseInImage_);

//        tf::Stamped<tf::Pose> topRightCorner;
        if(facing_.shelfType == Facing::ShelfType::STANDING) {
          facing_.topRightCorner = facing_.rightSeparator;
          facing_.topRightCorner.setOrigin(tf::Vector3(facing_.rightSeparator.getOrigin().x(),
                                               facing_.rightSeparator.getOrigin().y(),
                                               facing_.rightSeparator.getOrigin().z() + facing_.productDims.h));
        }
        else {
          facing_.topRightCorner = facing_.leftSeparator;
          facing_.topRightCorner.setOrigin(tf::Vector3(facing_.leftSeparator.getOrigin().x() - facing_.productDims.d / 2.0,
                                               facing_.leftSeparator.getOrigin().y(),
                                               facing_.leftSeparator.getOrigin().z() - facing_.productDims.h));
        }
        listener->waitForTransform(camInfo_.header.frame_id, facing_.topRightCorner.frame_id_, ros::Time(0), ros::Duration(2.0));
        listener->transformPose(camInfo_.header.frame_id,/* ros::Time(0), */facing_.topRightCorner,/* "map"*/ topRightCornerInImage_);


        listener->waitForTransform(flir_cam_info_.header.frame_id, facing_.topRightCorner.frame_id_, ros::Time(0), ros::Duration(2.0));
        listener->transformPose(flir_cam_info_.header.frame_id, facing_.topRightCorner, topRightCornerInFlirImage_);
        listener->transformPose(flir_cam_info_.header.frame_id, facing_.leftSeparator, leftSeparatorPoseInFlirImage_);
        listener->transformPose(flir_cam_info_.header.frame_id, facing_.rightSeparator, rightSeparatorPoseInFlirImage_);


        facing_.rect = calcRectInImage(separatorPoseInImage_, nextSeparatorPoseInImage_, topRightCornerInImage_, cam_info_);
        facing_.rect_hires_ = calcRectInImage(leftSeparatorPoseInFlirImage_, rightSeparatorPoseInFlirImage_, topRightCornerInFlirImage_, flir_cam_info_);
        listener->transformPose(camInfo_.header.frame_id, facing_.rightSeparator, nextSeparatorPoseInImage_);


//        if(saveImgFiles_) {
//          rs::ScopeTime scopeTime(OUT_FILENAME, "saveImgFiles", __LINE__);
//          std::fstream fstream;
//          std::stringstream filename;
//          filename << folderPath_ << "/gtin_" << facing_.gtin << "_" << camInfo_.header.stamp.toNSec();
//          fstream.open(filename.str() + "_meta.json", std::fstream::out);
//          fstream << "{\"dan\":" << facing_.gtin << ","
//                  << " \"rect\":{" << "\"x\":" << facing_.rect.x << ",\n"
//                  << "\"y\":" << facing_.rect.y << ",\n"
//                  << "\"h\":" << facing_.rect.height << ",\n"
//                  << "\"w\":" << facing_.rect.width << "}\n";
//          fstream << "}";
//          fstream.flush();
//          fstream.close();
//          cv::imwrite(filename.str() + "_rgb.png", rgb_);
//        }
      }
      catch(tf::TransformException &ex) {
        outError(ex.what());
        return UIMA_ERR_NONE;
      }

      Eigen::Affine3d eigenTransform;
      tf::transformTFToEigen(camToWorld_, eigenTransform);
      pcl::transformPointCloud<pcl::PointXYZRGBA>(*cloud_ptr_, *cloudFiltered_, eigenTransform);
      *cloud_transformed_ = * cloudFiltered_;

      filterCloud(facing_);

      clusterCloud(facing_.productDims.d, cloud_normals);


      addToCas(tcas, facing_);

      separatorPoints_->clear();
      return true;
    }
    else
      return false;
  }

  cv::Rect calcRectInImage(tf::Stamped<tf::Pose> separatorPose, tf::Stamped<tf::Pose> nextSeparatorPose,tf::Stamped<tf::Pose> topRightCorner, const sensor_msgs::CameraInfo &cam_info)
  {
    cv::Point2d bottomleftPoint = projection(separatorPose,cam_info);
    cv::Point2d toprightPOint = projection(topRightCorner,cam_info);
    cv::Point2d bottomRightPoint = projection(nextSeparatorPose,cam_info);

    cv::Rect rect(std::min(bottomleftPoint.x, toprightPOint.x),
                  std::min(bottomleftPoint.y, toprightPOint.y),
                  std::fabs(bottomleftPoint.x - bottomRightPoint.x), std::fabs(bottomleftPoint.y - toprightPOint.y));
    rect.x = std::max(0, rect.x);
    rect.y = std::max(0, rect.y);
    if((rect.x + rect.width) > rgb_.cols) {
      rect.width = rgb_.cols - rect.x;
    }
    if((rect.y + rect.height) > rgb_.rows) {
      rect.height = rgb_.rows - rect.y;
    }

    return rect;
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
    rs::SceneCas cas(tcas);

    cas.set("display_image",rgb_);

    return UIMA_ERR_NONE;
  }

  cv::Point2d projection(const tf::Stamped<tf::Pose> pose3D, sensor_msgs::CameraInfo cam_info)
  {
    std::vector<cv::Point3d> objectPoints;
    objectPoints.push_back(cv::Point3d(pose3D.getOrigin().x(), pose3D.getOrigin().y(), pose3D.getOrigin().z()));

    // Create the known projection matrix

    cv::Mat P(3, 4, cv::DataType<double>::type);
    //    P.data = *camInfo_.P.data();
    P.at<double>(0, 0) = cam_info.P[0];
    P.at<double>(1, 0) = cam_info.P[4];
    P.at<double>(2, 0) = cam_info.P[8];

    P.at<double>(0, 1) = cam_info.P[1];
    P.at<double>(1, 1) = cam_info.P[5];
    P.at<double>(2, 1) = cam_info.P[9];

    P.at<double>(0, 2) = cam_info.P[2];
    P.at<double>(1, 2) = cam_info.P[6];
    P.at<double>(2, 2) = cam_info.P[10];

    P.at<double>(0, 3) = cam_info.P[3];
    P.at<double>(1, 3) = cam_info.P[7];
    P.at<double>(2, 3) = cam_info.P[11];

    // Decompose the projection matrix into:
    cv::Mat K(3, 3, cv::DataType<double>::type); // intrinsic parameter matrix
    cv::Mat rvec(3, 3, cv::DataType<double>::type); // rotation matrix
    cv::Mat Thomogeneous(4, 1, cv::DataType<double>::type); // translation vector

    cv::decomposeProjectionMatrix(P, K, rvec, Thomogeneous);

    cv::Mat T(3, 1, cv::DataType<double>::type); // translation vector
    //    cv::convertPointsHomogeneous(Thomogeneous, T);
    T.at<double>(0) = 0.0;
    T.at<double>(1) = 0.0;
    T.at<double>(2) = 0.0;

    // Create zero distortion
    cv::Mat distCoeffs(4, 1, cv::DataType<double>::type);
    distCoeffs.at<double>(0) = 0;
    distCoeffs.at<double>(1) = 0;
    distCoeffs.at<double>(2) = 0;
    distCoeffs.at<double>(3) = 0;

    std::vector<cv::Point2d> projectedPoints;

    cv::Mat rvecR(3, 1, cv::DataType<double>::type); //rodrigues rotation matrix
    rvecR.at<double>(0) = 0.0;
    rvecR.at<double>(1) = 0.0;
    rvecR.at<double>(2) = 0.0;

    cv::projectPoints(objectPoints, rvecR, T, K, distCoeffs, projectedPoints);
    return projectedPoints[0];
  }

  void drawOnImage()
  {
    for(int j = 0; j < cluster_indices_.size(); ++j) {
      for(int i = 0; i < cluster_indices_[j].indices.size(); ++i) {
        int index = cluster_indices_[j].indices[i];
        rgb_.at<cv::Vec3b>(index) = rs::common::cvVec3bColors[j % rs::common::numberOfColors];
      }
    }

    //THE NICE WAY
    cv::Point leftSepInImage =  projection(separatorPoseInImage_, cam_info_);
    cv::Point rightSepInImage =  projection(nextSeparatorPoseInImage_,cam_info_);
    cv::Point origLeftSepInImage =  projection(originalSeparator1PoseImageFrame_,cam_info_);
    cv::Point origRightSepInImage =  projection(originalSeparator2PoseImageFrame_,cam_info_);

    if(leftSepInImage.y > camInfo_.height) leftSepInImage.y =  camInfo_.height - 2;
    if(rightSepInImage.y > camInfo_.height) rightSepInImage.y =  camInfo_.height - 2;

    outInfo("Left Sep image coords: " << leftSepInImage);
    outInfo("Right Sep image coords: " << rightSepInImage);
    cv::circle(rgb_, leftSepInImage, 5, cv::Scalar(255, 0, 0), 3);
    cv::circle(rgb_, rightSepInImage, 5, cv::Scalar(255, 0, 0), 3);

    cv::circle(rgb_, origLeftSepInImage, 5, cv::Scalar(0, 0, 255), 3);
    cv::circle(rgb_, origRightSepInImage, 5, cv::Scalar(0, 0, 255), 3);

    cv::rectangle(rgb_, facing_.rect, cv::Scalar(0, 255, 0));

    //TOOD: use image transport
//    cv_bridge::CvImage outImgMsgs;
//    outImgMsgs.header = camInfo_.header;
//    outImgMsgs.encoding = sensor_msgs::image_encodings::BGR8;
//    outImgMsgs.image = rgb_;
//    image_pub_.publish(outImgMsgs.toImageMsg());
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
    if(firstRun) {
      visualizer.addPointCloud(cloudFiltered_, cloudname);
      visualizer.addPointCloud(cloud_transformed_, cloudTransformedname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize / 2.0, cloudTransformedname);
    }
    else {
      visualizer.updatePointCloud(cloudFiltered_,  cloudname);
      visualizer.updatePointCloud(cloud_transformed_, cloudTransformedname);
      visualizer.removeAllShapes();
    }

    int idx = 0;
    for(auto &bb : cluster_boxes) {
      visualizer.addCube(bb.minPt.x, bb.maxPt.x, bb.minPt.y, bb.maxPt.y, bb.minPt.z, bb.maxPt.z, 1.0, 1.0, 1.0,
                         "box_" + std::to_string(idx));
      idx++;
    }
    visualizer.setRepresentationToWireframeForAllActors();
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(ProductCounter)
