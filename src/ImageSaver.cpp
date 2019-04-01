#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>
//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>

#include <ros/package.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>

using namespace uima;


class ImageSaver : public Annotator
{
private:
    sensor_msgs::CameraInfo cam_info_;
    std::string folder_path_;
public:

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    std::string packagePath = ros::package::getPath("rs_refills") + "/data_out";
    boost::posix_time::ptime posixTime = ros::Time::now().toBoost();
    std::string iso_time_str = boost::posix_time::to_iso_extended_string(posixTime);
    folder_path_ = packagePath + "/" + iso_time_str;
    outWarn(folder_path_);
    boost::filesystem::path path(folder_path_);
    if(!boost::filesystem::exists(path)) {
      outInfo("Creating folder: " << path.string());
      boost::filesystem::create_directory(path);
    }
    else {
      outWarn("How can this already exist?");
    }


    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
  {
      outInfo("process start");
      rs::StopWatch clock;
      cv::Mat rgb;
      cv::Rect facingRect;
      std::string gTinOfFacing;

      std::string testFileName = "";

      std::vector<std::string> modelImages;
      rs::SceneCas cas(tcas);
      rs::Scene scene = cas.getScene();
      cas.get(VIEW_COLOR_IMAGE, rgb);
      cas.get(VIEW_CAMERA_INFO, cam_info_);

      std::vector<rs::ObjectHypothesis> hyps;
      scene.identifiables.filter(hyps);
      outInfo("Found " << hyps.size() << " hypotheses.");
      for(auto &h : hyps) {
       std::vector<rs::Detection> detections;
       std::vector<rs::Classification> classification;
       h.annotations.filter(detections);
       h.annotations.filter(classification);

       if(detections.empty()) continue;
       if(classification.empty()) continue;
       rs::Detection &det = detections[0];
       rs::Classification &cl = classification[0];
       rs::ImageROI roi = h.rois();
       rs::conversion::from(roi.roi(), facingRect);

       std::fstream fstream;
       std::stringstream filename;
       filename << folder_path_ << "/"<< cam_info_.header.stamp.toNSec();
       fstream.open(filename.str() + "_meta.json", std::fstream::out);
       fstream << "{\"gtin_facing\": \"" << det.name() << "\","
               << " \"gtin_result\": \"" << cl.classname()<< "\","
               << " \"rect\":{" << "\"x\":" << facingRect.x << ",\n"
               << "\"y\":" << facingRect.y << ",\n"
               << "\"h\":" << facingRect.height << ",\n"
               << "\"w\":" << facingRect.width << "}\n";
       fstream << "}";
       fstream.flush();
       fstream.close();
       cv::imwrite(filename.str() + "_rgb.png", rgb);


      }

    return UIMA_ERR_NONE;
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(ImageSaver)