#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>
//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>

#include <opencv2/opencv.hpp>

//yaml
#include <yaml-cpp/yaml.h>

#include <ros/package.h>
#include <rs/DrawingAnnotator.h>

using namespace uima;


class MisplacedItemDetection : public DrawingAnnotator
{
private:
  float test_param;

  std::map<std::string, std::vector<std::string>> gtinToImages;
  YAML::Node yamlModelsConfig;

  cv::Ptr<cv::ORB> detector;

  cv::Mat disp_;

public:

  MisplacedItemDetection(): DrawingAnnotator(__func__)
  {
    detector = cv::ORB::create();
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {

    outInfo("initialize");
    std::string pathToRefillsModels = ros::package::getPath("refills_models");
    std::string pathToModelFile = pathToRefillsModels + "/models/model_data.yaml";

    yamlModelsConfig = YAML::LoadFile(pathToModelFile);

    for(YAML::const_iterator it = yamlModelsConfig.begin(); it != yamlModelsConfig.end(); ++it) {
      YAML::Node key = it->first;
      YAML::Node value = it->second;
      if(key.Type() == YAML::NodeType::Scalar) {
        std::string gtin  = key.as<std::string>();
        if(value.Type() == YAML::NodeType::Map) {
          for(YAML::const_iterator it2 = value.begin(); it2 != value.end(); ++it2) {
            if(it2->second.Type() == YAML::NodeType::Map) {
              std::string pathToImageFile, imageFileName;
              imageFileName = it2->second["lowres_filename_png"].as<std::string>();
              pathToImageFile = it2->second["lowres_model_path"].as<std::string>();
              gtinToImages[gtin].push_back(pathToRefillsModels + "/models/" + pathToImageFile + "/" + imageFileName);
            }
          }
        }
        else
          outError("value under gtin: " << gtin << "in yaml file is not a map!");
      }
      else {
        std::string msg = "Node's key should be scalar.";
        YAML::ParserException e(YAML::Mark::null_mark(), msg);
        throw e;
      }

    }/*
    for(auto m : gtinToImages) {
      outInfo(m.first);
      for(auto f : m.second)
        outInfo(f);
    }*/
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
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();
    cv::Mat rgb;
    cas.get(VIEW_COLOR_IMAGE_HD, rgb);
    std::vector<rs::ObjectHypothesis> hyps;
    scene.identifiables.filter(hyps);
    for(auto &h : hyps) {
      std::vector<rs::Detection> detections;
      h.annotations.filter(detections);
      if(detections.empty()) continue;
      rs::Detection &det = detections[0];
      if(det.source()  == "FacingDetection") {
        rs::ImageROI roi = h.rois();
        cv::Rect rect;
        rs::conversion::from(roi.roi_hires(), rect);
        cv::Mat facingImg = rgb(rect);
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        //keypoints for the facing
        detector->detectAndCompute(facingImg, cv::noArray(), keypoints1, descriptors1);

        std::vector<std::string> &modelImages = gtinToImages[det.name()];
        cv::Mat modelImage;
        for(auto mi : modelImages) {
          modelImage = cv::imread(mi);
          detector->detectAndCompute(modelImage, cv::noArray(), keypoints2, descriptors2);
          break;
        }
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher->match(descriptors1, descriptors2, matches);
        std::sort(matches.begin(),matches.end(),[](cv::DMatch a, cv::DMatch b){
                return  a.distance < b.distance;
        });

        cv::drawMatches(facingImg, keypoints1, modelImage, keypoints2, matches, disp_, cv::Scalar::all(-1),
                        cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
      }
    }
    return UIMA_ERR_NONE;
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    disp = disp_.clone();
  }

};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(MisplacedItemDetection)
