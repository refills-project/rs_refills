#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>
//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>

#include <opencv2/opencv.hpp>

//yaml
#include <yaml-cpp/yaml.h>
#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>

#include <ros/package.h>
#include <rs/DrawingAnnotator.h>

#include <boost/filesystem.hpp>
using namespace uima;

namespace bfs = boost::filesystem;

class MisplacedItemDetection : public DrawingAnnotator
{
private:
  float test_param;

  std::map<std::string, std::vector<std::string>> gtinToImages_;
  std::vector<std::pair<std::string, std::string>> testFiles_;
  YAML::Node yamlModelsConfig;

  cv::Ptr<cv::ORB> detector;

  cv::Mat disp_;

  inline static int imgIdx_ = 0;
  bool testMode_;
public:

  MisplacedItemDetection(): DrawingAnnotator(__func__), testMode_(false)
  {
    detector = cv::ORB::create();
  }

  void readInfoFromJson(const std::string &pathToJson, cv::Rect &rect, std::string &gtin)
  {
    std::ifstream ifs(pathToJson);
    rapidjson::IStreamWrapper isw(ifs);
    rapidjson::Document d;
    d.ParseStream(isw);

    if(d.HasMember("rect")) {
      rect.x =   d["rect"]["x"].GetInt();
      rect.y = d["rect"]["y"].GetInt();
      rect.height  = d["rect"]["h"].GetInt();
      rect.width = d["rect"]["w"].GetInt();
    }
    gtin = std::to_string(d["dan"].GetInt64());
  }

  cv::Mat getMaskForModelImage(const cv::Mat &imgGrey)
  {
      cv::Mat bin;
      cv::threshold(imgGrey, bin, 10, 255,cv::THRESH_BINARY);
      cv::Mat element = cv::getStructuringElement(0, cv::Size(11,11),cv::Point(5,5));
      cv::morphologyEx(bin, bin, cv::MORPH_ERODE,element);
//      cv::Ptr<cv::SimpleBlobDetector> = cv::SimpleBlobDetector::create();
      return bin;
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
              gtinToImages_[gtin].push_back(pathToRefillsModels + "/models/" + pathToImageFile + "/" + imageFileName);
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

    }
    if(ctx.isParameterDefined("testMode")) {
      ctx.extractValue("testMode", testMode_);
      if(testMode_) {
        std::string pathToTestImgs = ros::package::getPath("rs_refills") + "/test_data/";
        bfs::path bPathToImgs(pathToTestImgs);
        bfs::directory_iterator dIt(bPathToImgs);
        while(dIt != bfs::directory_iterator()) {

          bfs::path pathToFile(*dIt);
          if(pathToFile.extension() == ".png") {
            std::string pathToPng = pathToFile.string();
            size_t pos = pathToFile.string().find("_rgb");
            std::string pathToJson  = pathToFile.string().substr(0, pos) + "_meta.json";

            testFiles_.push_back(std::pair<std::string, std::string>(pathToPng, pathToJson));
            outInfo(pathToPng);
            outInfo(pathToJson);
          }
          dIt++;
        }
      }
    }
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
    cv::Mat rgb;

    cv::Mat facingImg;
    cv::Rect facingRect;
    std::string gTinOfFacing;

    if(!testMode_) {
      rs::SceneCas cas(tcas);
      rs::Scene scene = cas.getScene();
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

          rs::conversion::from(roi.roi_hires(), facingRect);
          facingImg = rgb(facingRect);
          gTinOfFacing = det.name();
        }
      }
    }
    else {
      std::pair<std::string, std::string> &testFilePair = testFiles_[imgIdx_++];
      imgIdx_ = imgIdx_ % testFiles_.size();
      rgb = cv::imread(testFilePair.first, cv::IMREAD_GRAYSCALE);
      readInfoFromJson(testFilePair.second, facingRect, gTinOfFacing);
      facingImg = rgb(facingRect);
    }

    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat descriptors1, descriptors2;

    //keypoints for the facing
    detector->detectAndCompute(facingImg, cv::noArray(), keypoints1, descriptors1);

    cv::imwrite("facing.png", facingImg);

    outInfo("Detected " << keypoints1.size() << " keypoints in facjng image");

    std::vector<std::string> &modelImages = gtinToImages_[gTinOfFacing];

    cv::Mat modelImage;
    for(auto mi : modelImages) {
      outInfo("Gtin: " << gTinOfFacing << " model image: " << mi);
      modelImage = cv::imread(mi, cv::IMREAD_GRAYSCALE);
      cv::Mat modelMask = getMaskForModelImage(modelImage);
//      disp_ = modelMask.clone();
      cv::imwrite("model.png", modelImage);
      detector->detectAndCompute(modelImage, modelMask, keypoints2, descriptors2);
      break;
    }

    outInfo("Detected " << keypoints2.size() << " keypoints in model image");

    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
    std::vector<cv::DMatch> matches;
    matcher->match(descriptors1, descriptors2, matches);
    std::sort(matches.begin(), matches.end(), [](cv::DMatch a, cv::DMatch b) {
      return  a.distance < b.distance;
    });

    int idx = 0 ;
    for(auto m : matches) {
      outInfo("Match nr" << idx++ <<" dist: "<<  m.distance);

    }
    std::vector<char> mask(matches.size(), 0);
    if(mask.size() > 50) {
      for(int i = 0; i < 50; ++i) {
        mask[i] = 1;
      }
    }

    cv::drawMatches(facingImg, keypoints1, modelImage, keypoints2, matches, disp_, cv::Scalar::all(-1),
                    cv::Scalar::all(-1), mask, cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    return UIMA_ERR_NONE;
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    disp = disp_.clone();
  }

};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(MisplacedItemDetection)
