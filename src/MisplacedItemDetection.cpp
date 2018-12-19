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

  int imgIdx_, modelIdx_;
  bool testMode_;


  enum {
    CANNY,
    MASK,
    BIN,
    MATCHES,
    HIST
  } displayMode;

  cv::Mat canny_, bin_, mask_, hist_;

  std::stringstream positiveMatch, negativeMatch;
public:

  MisplacedItemDetection(): DrawingAnnotator(__func__), imgIdx_(0), modelIdx_(0), testMode_(false), displayMode(BIN)
  {
    detector = cv::ORB::create();
  }

  bool callbackKey(const int key, const Source source)
  {
    switch(key) {
    case 'b':
    case 'B':
      displayMode = BIN;
      return true;
    case 'c':
    case 'C':
      displayMode = CANNY;
      return true;
    case 'm':
    case 'M':
      displayMode = MASK;
      return true;
    case 'O':
    case 'o':
      displayMode = MATCHES;
      return true;
    case 'H':
    case 'h':
      displayMode = HIST;
      return true;
    }
    return false;
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
    cv::Mat bin, canny;
    cv::threshold(imgGrey, bin, 10, 255, cv::THRESH_BINARY);
    cv::Mat element = cv::getStructuringElement(0, cv::Size(11, 11), cv::Point(5, 5));
    cv::morphologyEx(bin, bin, cv::MORPH_ERODE, element);
    bin_ = bin.clone();
    int cannyThreshold = 10;
    cv::Canny(bin, canny, cannyThreshold, cannyThreshold * 2);
    cv::morphologyEx(canny, canny, cv::MORPH_DILATE, element);
    canny_ = canny.clone();
    std::vector<std::vector<cv::Point> > contours;
    std::vector<std::vector<cv::Point> > contoursToKeep;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(canny, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    outInfo("Found " << contours.size() << " contours");

    if(contours.size() == 0) {
      mask_ = cv::Mat::ones(imgGrey.rows, imgGrey.cols, CV_8U);
      return mask_;
    }

    cv::Mat newImg = cv::Mat::zeros(imgGrey.rows, imgGrey.cols, CV_8U);
    for(int idx = 0; idx < contours.size(); idx ++) {
      double area = cv::contourArea(contours[idx]);
      if(area > 0.05 * imgGrey.cols * imgGrey.rows && hierarchy[idx][3] == -1) {
        contoursToKeep.push_back(contours[idx]);
      }
    }

    cv::drawContours(newImg, contoursToKeep, -1, cv::Scalar(255));
    cv::fillPoly(newImg, contoursToKeep, cv::Scalar(255));

    cv::morphologyEx(newImg, newImg, cv::MORPH_ERODE, element);
    mask_ = newImg.clone();
    return mask_;
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


  /**
   * @brief matchModelToFacing perform matching and getSimilarity score
   * @return
   */
  double matchModelToFacing()
  {

  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  cv::Mat calcHistogram(cv::Mat &imgGrey, cv::Mat mask = cv::Mat())
  {
    /// Establish the number of bins
    int histSize = 64;
    /// Set the ranges ( for B,G,R) )
    float range[] = { 0, 256 } ;
    const float *histRange = { range };

    cv::Mat histModel, histModelNormed;
    cv::calcHist(&imgGrey, 1, 0, mask, histModel, 1, &histSize, &histRange, true, false);
    cv::normalize(histModel, histModelNormed, 0, 1.0, cv::NORM_MINMAX, -1, cv::Mat());


    int hist_w = 512;
    int hist_h = 100;
    int bin_w = cvRound((double) hist_w / histSize);
    cv::normalize(histModel, histModel, 0, hist_h, cv::NORM_MINMAX, -1, cv::Mat());

    cv::Mat histImage(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));

    for(int i = 1; i < histSize; i++) {
      cv::line(histImage, cv::Point(bin_w * (i - 1), hist_h - cvRound(histModel.at<float>(i - 1))),
               cv::Point(bin_w * (i), hist_h - cvRound(histModel.at<float>(i))),
               cv::Scalar(255, 0, 0), 2, 8, 0);

    }
    hist_.push_back(histImage);
    return histModelNormed;
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    rs::StopWatch clock;
    cv::Mat rgb;
    hist_ = cv::Mat();
    cv::Mat facingImg;
    cv::Rect facingRect;
    std::string gTinOfFacing;

    std::string testFileName = "";

    std::vector<std::string> modelImages;
    if(!testMode_) {
      rs::SceneCas cas(tcas);
      rs::Scene scene = cas.getScene();
      cas.get(VIEW_COLOR_IMAGE_HD, rgb);
      std::vector<rs::ObjectHypothesis> hyps;
      for(auto &h : hyps) {
        std::vector<rs::Detection> detections;
        h.annotations.filter(detections);
        if(detections.empty()) continue;

        rs::Detection &det = detections[0];
        det.confidence.set(0.77);

        if(det.source()  == "FacingDetection") {
          rs::ImageROI roi = h.rois();

          rs::conversion::from(roi.roi_hires(), facingRect);
          facingImg = rgb(facingRect);
          gTinOfFacing = det.name();
          modelImages = gtinToImages_[gTinOfFacing];
        }
      }
    }
    else {
      std::pair<std::string, std::string> &testFilePair = testFiles_[imgIdx_];
      testFileName = testFilePair.first;
      rgb = cv::imread(testFilePair.first, cv::IMREAD_GRAYSCALE);
      readInfoFromJson(testFilePair.second, facingRect, gTinOfFacing);
      facingImg = rgb(facingRect);

      std::map<std::string, std::vector<std::string> >::iterator it = gtinToImages_.begin();
      it = std::next(it, modelIdx_++);
      if(modelIdx_ == gtinToImages_.size()) {
        modelIdx_ = 0;
        imgIdx_++;
      }
      std::string modelGtin = it->first;
      modelImages = it->second;
    }

    //    bfs::path savefolderPath(ros::package::getPath("rs_refills") + "/test_results/" + gTinOfFacing + "/");
    //    if(!bfs::exists(savefolderPath)) boost::filesystem::create_directory(savefolderPath);

    cv::Mat facingHist = calcHistogram(facingImg);
    cv::GaussianBlur(facingImg, facingImg, cv::Size(9, 9), 0);

    std::vector<cv::KeyPoint> facingKeypoints, modelKeypoints;
    std::vector<cv::Point2f> goodKp1, goodKp2, transformedKps;
    cv::Mat descriptors1, descriptors2;

    //keypoints for the facing
    detector->detectAndCompute(facingImg, cv::noArray(), facingKeypoints, descriptors1);
    outInfo("Detected " << facingKeypoints.size() << " keypoints in facing image:" << testFileName);

    cv::Mat modelImage;
    cv::Mat modelHist;
    for(auto mi : modelImages) {
      outInfo("Gtin: " << gTinOfFacing << " model image: " << mi);
      modelImage = cv::imread(mi, cv::IMREAD_GRAYSCALE);
      cv::Mat modelMask = getMaskForModelImage(modelImage);
      modelHist = calcHistogram(modelImage, modelMask);
      cv::GaussianBlur(modelImage, modelImage, cv::Size(9, 9), 0);
      detector->detectAndCompute(modelImage, modelMask, modelKeypoints, descriptors2);
      break;
    }
    outInfo("Detected " << modelKeypoints.size() << " keypoints in model image");

    double histDist = cv::compareHist(facingHist, modelHist, CV_COMP_CORREL);

    cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);
    std::vector<cv::DMatch> matches;


    outInfo("Size of descriptors: " << descriptors1.size() << " D2: " << descriptors2.size());
    if(!descriptors1.empty() && !descriptors2.empty()) {
      matcher->match(descriptors1, descriptors2, matches);
      std::sort(matches.begin(), matches.end(), [](cv::DMatch a, cv::DMatch b) {
        return  a.distance < b.distance;
      });
    }
    std::vector<char> mask(matches.size(), 0);
    for(int i = 0; i < matches.size() && matches[i].distance <= 60; ++i) {
      mask[i] = 1;
    }

    float avgDist = 0.0;
    uint8_t mi = 0;
    for(; mi < matches.size() && matches[mi].distance <= 60   ; ++mi) {
      outInfo(matches[mi].distance);
      avgDist += (matches[mi].distance);
      goodKp1.push_back(facingKeypoints[matches[mi].queryIdx].pt);
      goodKp2.push_back(modelKeypoints[matches[mi].trainIdx].pt);
    }

    cv::Mat homograpy;
    if(!goodKp1.size() >= 4 && !goodKp1.size() >= 4) {
      homograpy = cv::findHomography(goodKp1, goodKp2, CV_RANSAC);
      cv::perspectiveTransform(goodKp1, transformedKps, homograpy);
    }

    float sum = 0.0;
    for(int j = 0; j < transformedKps.size(); ++j) {
      sum += std::sqrt((transformedKps[j].x - goodKp2[j].x) * (transformedKps[j].x - goodKp2[j].x)  + (transformedKps[j].y - goodKp2[j].y) * (transformedKps[j].y - goodKp2[j].y));
    }
    sum = sum / transformedKps.size() + 1;
    outInfo("Transform error:" << sum);

    //    std::ofstream histFile;
    //    if(modelGtin == gTinOfFacing) {
    //      histFile.open(savefolderPath.string() + "01match.csv", std::ostream::out);
    //    }
    //    else {
    //      histFile.open(savefolderPath.string() + modelGtin + ".csv", std::ostream::out);
    //    }

    //    for(auto m : matches) {
    //      histFile << m.distance / 256 << ",";
    //    }
    //    histFile.close();

    avgDist /= mi;
    outInfo("Average dist of best 20 matches: " << avgDist);
    outInfo("Histogram correlation: " << histDist);

    //    if(modelGtin == gTinOfFacing) {
    //      positiveMatch << sum << ",";
    //      outError("THIS IS THE REAL MATCH");
    //    }
    //    else {
    //      negativeMatch << sum << ",";
    //    }

    cv::drawMatches(facingImg, facingKeypoints, modelImage, modelKeypoints, matches, disp_, cv::Scalar::all(-1),
                    cv::Scalar::all(-1), mask, cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    if(imgIdx_ == testFiles_.size()) {
      outError("DONE!!!!!!!!1");
      std::ofstream ofp, ofn;
      ofp.open("outpositive.csv", std::ofstream::out);
      ofn.open("outnegative.csv", std::ofstream::out);
      ofp << positiveMatch.str();
      ofn << negativeMatch.str();
      ofp.close();
      ofn.close();
      ros::shutdown();
      exit(0);
    }

    return UIMA_ERR_NONE;
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    switch(displayMode) {
    case BIN:
      disp = bin_.clone();
      break;
    case CANNY:
      disp = canny_.clone();
      break;
    case MASK:
      disp = mask_.clone();
      break;
    case MATCHES:
      disp = disp_.clone();
      break;
    case HIST:
      disp = hist_.clone();
      break;
    default:
      disp = disp_.clone();
      break;
    }
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(MisplacedItemDetection)
