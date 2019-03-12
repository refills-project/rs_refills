#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include <ros/package.h>

#include <refills_msgs/SeparatorArray.h>

//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>
#include <rs_refills/types/all_types.h>
#include <rs_refills/common_structs.h>

//KnowRob
#include <json_prolog/prolog.h>

//rapidjson
#include <rapidjson/document.h>
#include <rapidjson/rapidjson.h>

//pcl
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>


using namespace uima;


class FacingAnnotator : public DrawingAnnotator
{

private:
  ros::NodeHandle nh_;

  ros::Subscriber separator_subscriber_;
  std::shared_ptr<tf::TransformListener> listener_;

  std::mutex mtx_;
  bool save_img_files_;
  int idx_;
  std::string folder_path_;

  std::string local_frame_name_;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr separator_points_;
  tf::StampedTransform camToWorld_;

  //for visualization purposes
  cv::Mat dispImg_;

  sensor_msgs::CameraInfo cam_info_;
  tf::Stamped<tf::Pose> left_separator_pose_in_image_, right_separator_pose_in_image_;
  tf::Stamped<tf::Pose> original_left_separator_pose_img_frame_, original_right_separator_pose_img_frame_;
  tf::Stamped<tf::Pose> top_right_corner_in_image_;

public:

  FacingAnnotator(): DrawingAnnotator(__func__), nh_("~"),save_img_files_(false), idx_(0)
  {
    //create a new folder for saving images into
    std::string packagePath = ros::package::getPath("rs_refills") + "/data_out";
    boost::posix_time::ptime posixTime = ros::Time::now().toBoost();
    std::string iso_time_str = boost::posix_time::to_iso_extended_string(posixTime);
    folder_path_ = packagePath + "/" + iso_time_str;
    outWarn(folder_path_);
    boost::filesystem::path path(folder_path_);
    if(!boost::filesystem::exists(path))
    {
      outInfo("Creating folder: " << path.string());
      boost::filesystem::create_directory(path);
    }
    else
    {
      outWarn("How can this already exist?");
    }
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    separator_subscriber_ = nh_.subscribe("/separator_marker_detector_node/data_out", 50, &FacingAnnotator::separatorCb, this);
    listener_ = std::make_shared<tf::TransformListener>(nh_, ros::Duration(10.0));
    ctx.extractValue("saveImgFiles", save_img_files_);

    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  void separatorCb(const refills_msgs::SeparatorArrayPtr &msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if(local_frame_name_ != "")
    {
      for(auto m : msg->separators)
      {
        tf::Stamped<tf::Pose> pose_stamped, poseBase;
        tf::poseStampedMsgToTF(m.separator_pose, pose_stamped);
        try
        {
          listener_->waitForTransform(local_frame_name_, pose_stamped.frame_id_, pose_stamped.stamp_, ros::Duration(1.0));
          listener_->transformPose(local_frame_name_, pose_stamped.stamp_, pose_stamped, pose_stamped.frame_id_, poseBase);

          pcl::PointXYZRGBA pt;
          pt.x = static_cast<float>(poseBase.getOrigin().x());
          pt.y = static_cast<float>(poseBase.getOrigin().y());
          pt.z = static_cast<float>(poseBase.getOrigin().z());
          separator_points_->points.push_back(pt);
        }
        catch(tf::TransformException ex)
        {
          outWarn(ex.what());
        }
      }
    }
  }

  bool getFacingInformation(rs_refills::ProductFacing &facing, const std::string facing_id, uima::CAS &tcas)
  {
    //owl_instance_from_class(shop:'ProductWithAN377954',I),object_dimensions(I,D,W,H).
    MEASURE_TIME;
    std::stringstream plQuery;
    json_prolog::Prolog pl;
    plQuery << "shelf_facing(F,'" << facing_id << "'),shelf_layer_standing(F).";
    try
    {
      outInfo("Asking query: " << plQuery.str());
      json_prolog::PrologQueryProxy bdgs = pl.query(plQuery.str());
      rs::Facing::ShelfType shelf_type;
      if(bdgs.begin() == bdgs.end())
      {
        outInfo("facing is on a mounting hanging ");
        shelf_type = rs::Facing::ShelfType::HANGING;
      }
      else
      {
        outInfo("Standing shelf");
        shelf_type = rs::Facing::ShelfType::STANDING;

      }

      facing.shelfType.set(rs::getTextFromEnum(static_cast<int>(shelf_type)));
      facing.facingId.set(facing_id);

      tf::Stamped<tf::Pose> leftSepPose, rightSepPose;

      if(shelf_type == rs::Facing::ShelfType::STANDING)
      {
        //get the left and Right separators:
        plQuery.str(std::string(""));
        plQuery << "rdf_has('" << facing_id << "', shop:leftSeparator, L), object_perception_affordance_frame_name(L,LFrameName),"
                << "rdf_has('" << facing_id << "', shop:rightSeparator,R), object_perception_affordance_frame_name(R,RFrameName).";
        outInfo("Asking query: " << plQuery.str());
        bdgs = pl.query(plQuery.str());
        if(bdgs.begin() == bdgs.end())
        {
          outError("No results found the left and right separator. are you sure this is the right facing type?");
          return false;
        }
        std::string leftSepTFId, rightSepTFId;
        for(auto bdg : bdgs)
        {
          leftSepTFId = bdg["LFrameName"].toString();
          leftSepTFId = leftSepTFId.substr(1, leftSepTFId.size() - 2);
          rightSepTFId = bdg["RFrameName"].toString();
          rightSepTFId = rightSepTFId.substr(1, rightSepTFId.size() - 2);
          break;
        }

        tf::StampedTransform leftSep, rightSep;
        listener_->waitForTransform(local_frame_name_, leftSepTFId, ros::Time(0), ros::Duration(2.0));
        listener_->lookupTransform(local_frame_name_, leftSepTFId, ros::Time(0), leftSep);
        listener_->waitForTransform(local_frame_name_, rightSepTFId, ros::Time(0), ros::Duration(2.0));
        listener_->lookupTransform(local_frame_name_, rightSepTFId, ros::Time(0), rightSep);

        tf::Stamped<tf::Pose> temp1, temp2;
        temp1.setData(leftSep);
        temp2.setData(rightSep);
        listener_->transformPose(cam_info_.header.frame_id, temp1, original_left_separator_pose_img_frame_);
        listener_->transformPose(cam_info_.header.frame_id, temp2, original_right_separator_pose_img_frame_);

        //try to fix separator positions based on current detections;
        if(separator_points_->size() > 2)
        {
          outInfo("Using current detection of separators to fix positions");
          pcl::PointXYZRGBA s1, s2;
          s1.x = static_cast<float>(leftSep.getOrigin().x());
          s1.y = static_cast<float>(leftSep.getOrigin().y());
          s1.z = static_cast<float>(leftSep.getOrigin().z());

          s2.x = static_cast<float>(rightSep.getOrigin().x());
          s2.y = static_cast<float>(rightSep.getOrigin().y());
          s2.z = static_cast<float>(rightSep.getOrigin().z());

          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sep_points_copy(new pcl::PointCloud<pcl::PointXYZRGBA>(*separator_points_));
          pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
          tree->setInputCloud(sep_points_copy);
          std::vector<int> pointIdxRadiusSearch;
          std::vector<float> pointRadiusSquaredDistance;
          if(tree->radiusSearch(s1, 0.04, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
          {
            outInfo("Found " << pointIdxRadiusSearch.size() << " close to separator 1");
            pcl::PointXYZRGBA np = sep_points_copy->points[pointIdxRadiusSearch[0]];
            leftSep.setOrigin(tf::Vector3(np.x, np.y, np.z));
            sep_points_copy->points.erase(sep_points_copy->points.begin() + pointIdxRadiusSearch[0]);
          }

          tree->setInputCloud(sep_points_copy);
          if(tree->radiusSearch(s2, 0.04, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
          {
            outInfo("Found " << pointIdxRadiusSearch.size() << " close to separator 2");
            pcl::PointXYZRGBA np = sep_points_copy->points[pointIdxRadiusSearch[0]];
            rightSep.setOrigin(tf::Vector3(np.x, np.y, np.z));
          }
        }
        temp1.setData(leftSep);
        temp2.setData(rightSep);
        listener_->transformPose(cam_info_.header.frame_id, temp1, left_separator_pose_in_image_);
        listener_->transformPose(cam_info_.header.frame_id, temp2, right_separator_pose_in_image_);

        tf::Stamped<tf::Pose> topRightCorner;
        if(shelf_type == rs::Facing::ShelfType::STANDING)
        {
          topRightCorner.setData(rightSep);
          topRightCorner.setOrigin(tf::Vector3(rightSep.getOrigin().x(),
                                               rightSep.getOrigin().y(),
                                               rightSep.getOrigin().z() + facing.height()));
        }
        else
        {
          topRightCorner.setData(leftSep);
          topRightCorner.setOrigin(tf::Vector3(leftSep.getOrigin().x() - facing.depth() / 2.0,
                                               leftSep.getOrigin().y(),
                                               leftSep.getOrigin().z() - facing.height()));
        }

        listener_->waitForTransform(cam_info_.header.frame_id, topRightCorner.frame_id_, ros::Time(0), ros::Duration(2.0));
        listener_->transformPose(cam_info_.header.frame_id,topRightCorner, top_right_corner_in_image_);

        cv::Rect facing_roi = calcRectInImage(left_separator_pose_in_image_, top_right_corner_in_image_);

        if(save_img_files_)
        {
          rs::ScopeTime scopeTime(OUT_FILENAME, "saveImgFiles", __LINE__);
          std::fstream fstream;
          std::stringstream filename;
          filename << folder_path_ << "/gtin_" << facing.productId() << "_" << cam_info_.header.stamp.toNSec();
          fstream.open(filename.str() + "_meta.json", std::fstream::out);
          fstream << "{\"dan\":" << facing.productId() << ","
                  << " \"rect\":{" << "\"x\":" << facing_roi.x << ",\n"
                  << "\"y\":" <<facing_roi.y << ",\n"
                  << "\"h\":" <<facing_roi.height << ",\n"
                  << "\"w\":" <<facing_roi.width << "}\n";
          fstream << "}";
          fstream.flush();
          fstream.close();
          cv::imwrite(filename.str() + "_rgb.png", dispImg_);
        }

        cv::rectangle(dispImg_, facing_roi, cv::Scalar(0, 255, 0));
        facing.facing_roi.set(rs::conversion::to(tcas, facing_roi));
        leftSepPose.setData(leftSep);
        rightSepPose.setData(rightSep);
      }
      else if(shelf_type == rs::Facing::ShelfType::HANGING)
      {
        plQuery.str(std::string(""));
        plQuery << "rdf_has('" << facing_id << "', shop:mountingBarOfFacing, M), object_perception_affordance_frame_name(M,MFrameName).";
        outInfo("Asking query: " << plQuery.str());
        bdgs = pl.query(plQuery.str());
        if(bdgs.begin() == bdgs.end())
        {
          outError("This Facing has no mountint Bar...WTF");
          return false;
        }
        std::string mountingBarTFId;
        for(auto bdg : bdgs)
        {
          mountingBarTFId = bdg["MFrameName"].toString();
          mountingBarTFId = mountingBarTFId.substr(1, mountingBarTFId.size() - 2);
          break;
        }
        tf::StampedTransform mountingBar;
        listener_->lookupTransform(local_frame_name_, mountingBarTFId, ros::Time(0), mountingBar);
        leftSepPose.setData(mountingBar);
      }

      facing.leftSeparatorPose.set(rs::conversion::to(tcas, leftSepPose));
      facing.rightSeparatorPose.set(rs::conversion::to(tcas, rightSepPose));

      //get dimenstions and product type of facing
      plQuery.str(std::string());
      plQuery << "shelf_facing_product_type('" << facing_id << "', P),"
              << "owl_class_properties(P,shop:articleNumberOfProduct,AN),"
              << "comp_facingWidth('" << facing_id << "',literal(type(_, W_XSD))),atom_number(W_XSD,W),"
              << "comp_facingHeight('" << facing_id << "',literal(type(_, H_XSD))),atom_number(H_XSD,H)."
              << "comp_facingDepth('" << facing_id << "',literal(type(_, D_XSD))),atom_number(D_XSD,D).";
      outInfo("Asking query: " << plQuery.str());
      bdgs = pl.query(plQuery.str());
      if(bdgs.begin() == bdgs.end())
      {
        outError("Facing: " << facing_id << " has no width, height, or product type defined");
        return false;
      }
      for(auto bdg : bdgs)
      {
        facing.width.set(bdg["W"]);
        facing.height.set(bdg["H"]);
        facing.depth.set(bdg["D"]);
        facing.productId.set(bdg["P"].toString());
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

  /**
   * @brief parseQuery
   * @param[in] tcas tha cas.
   * @param[out] facing_ID id of the facing that we extract
   * @return true on success, false if query is not appropriate
   */
  bool parseQuery(CAS &tcas, std::string &facing_ID)
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
      if(!dQuery.HasMember("facing")) return false;
      facing_ID = dQuery["facing"].GetString();

      if(!dQuery.HasMember("location"))
      {
        outError("No location specified in query! Facings will not be detected");
        return false;
      };
      local_frame_name_ = dQuery["location"].GetString();
    }
    else
      return false;
    return true;
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    cas.get(VIEW_CAMERA_INFO, cam_info_);
    cas.get(VIEW_COLOR_IMAGE, dispImg_);

    std::string facing_id;
    if(!parseQuery(tcas, facing_id))
      return UIMA_ERR_NONE;

    rs_refills::ProductFacing product_facing = rs::create<rs_refills::ProductFacing>(tcas);
    if(!getFacingInformation(product_facing, facing_id, tcas))
      outError("Could not get facing information for facing_id: " << facing_id);

    scene.identifiables.append(product_facing);

    //don't clean separators every time
    if(idx_++ % 4 == 0)
    {
      idx_ = 0;
      separator_points_->clear();
    }
    return UIMA_ERR_NONE;
  }

  cv::Point2d projection(const tf::Stamped<tf::Pose> pose3D, const sensor_msgs::CameraInfo &cam_info)
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

  cv::Rect calcRectInImage(tf::Stamped<tf::Pose> separatorPose, tf::Stamped<tf::Pose> topRightCorner)
  {
    cv::Point2d bottomleftPoint = projection(separatorPose,cam_info_);
    cv::Point2d toprightPOint = projection(topRightCorner,cam_info_);
    cv::Rect rect(std::min(bottomleftPoint.x, toprightPOint.x),
                  std::min(bottomleftPoint.y, toprightPOint.y),
                  std::fabs(bottomleftPoint.x - toprightPOint.x), std::fabs(bottomleftPoint.y - toprightPOint.y));
    rect.x = std::max(0, rect.x);
    rect.y = std::max(0, rect.y);
    if((rect.x + rect.width) > dispImg_.cols)
      rect.width = dispImg_.cols - rect.x;
    if((rect.y + rect.height) > dispImg_.rows)
      rect.height = dispImg_.rows - rect.y;
    return rect;
  }


  void drawWithLock(cv::Mat &disp)
  {
    cv::Point leftSepInImage =  projection(left_separator_pose_in_image_,cam_info_);
    cv::Point rightSepInImage =  projection(right_separator_pose_in_image_,cam_info_);
    cv::Point origLeftSepInImage =  projection(original_left_separator_pose_img_frame_,cam_info_);
    cv::Point origRightSepInImage =  projection(original_right_separator_pose_img_frame_,cam_info_);

    if(leftSepInImage.y > cam_info_.height) leftSepInImage.y =  cam_info_.height - 2;
    if(rightSepInImage.y > cam_info_.height) rightSepInImage.y =  cam_info_.height - 2;

    outInfo("Left Sep image coords: " << leftSepInImage);
    outInfo("Right Sep image coords: " << rightSepInImage);
    cv::circle(dispImg_, leftSepInImage, 5, cv::Scalar(255, 0, 0), 3);
    cv::circle(dispImg_, rightSepInImage, 5, cv::Scalar(255, 0, 0), 3);

    cv::circle(dispImg_, origLeftSepInImage, 5, cv::Scalar(0, 0, 255), 3);
    cv::circle(dispImg_, origRightSepInImage, 5, cv::Scalar(0, 0, 255), 3);
    disp = dispImg_.clone();

  }

};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(FacingAnnotator)
