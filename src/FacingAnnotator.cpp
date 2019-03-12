#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

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
  int idx_;

  std::string local_frame_name_;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr separator_points_;

public:

  FacingAnnotator(): DrawingAnnotator(__func__), nh_("~"), idx_(0)
  {
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    separator_subscriber_ = nh_.subscribe("/separator_marker_detector_node/data_out", 50, &FacingAnnotator::separatorCb, this);
    listener_ = std::make_shared<tf::TransformListener>(nh_, ros::Duration(10.0));
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


  bool parseQuery(CAS &tcas, std::string &facing_ID)// tf::Stamped<tf::Pose> &pose, std::string &shelf_type, float &facingWidth)
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

  void drawWithLock(cv::Mat &disp)
  {

  }

};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(FacingAnnotator)
