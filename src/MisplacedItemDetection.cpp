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

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <refills_msgs/ProductIdentificationAction.h>

#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>


//image_transport
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace uima;

namespace bfs = boost::filesystem;

class MisplacedItemDetection : public DrawingAnnotator
{
private:
  float test_param;

  std::map<std::string, std::vector<std::string>> gtinToImages_;
  std::vector<std::pair<std::string, std::string>> testFiles_;
  YAML::Node yamlModelsConfig;

  sensor_msgs::CameraInfo cam_info_;

  cv::Ptr<cv::ORB> detector;

  cv::Mat disp_, model_, disp_img_;

  int imgIdx_, modelIdx_;
  bool testMode_, local_implementation_;


  ros::NodeHandle nodeHandle_;
  image_transport::Publisher image_pub_;
  image_transport::ImageTransport it_;

  enum {
    CANNY,
    MASK,
    BIN,
    MATCHES,
    HIST,
    MODEL
  } displayMode;

  cv::Mat canny_, bin_, mask_, hist_;

  std::stringstream positiveMatch, negativeMatch;
  actionlib::SimpleActionClient<refills_msgs::ProductIdentificationAction> *ac;
public:

  MisplacedItemDetection(): DrawingAnnotator(__func__), imgIdx_(0), modelIdx_(0), testMode_(false), displayMode(BIN), nodeHandle_("~"), it_(nodeHandle_)
  {
    detector = cv::ORB::create();
    canny_ = cv::Mat::zeros(640, 480, CV_8U);
    bin_ = cv::Mat::zeros(640, 480, CV_8U);
    mask_ = cv::Mat::zeros(640, 480, CV_8U);
    hist_ = cv::Mat::zeros(640, 480, CV_8UC3);
    disp_ = cv::Mat::zeros(640, 480, CV_8UC3);
    model_ = cv::Mat::zeros(640, 480, CV_8UC3);


    image_pub_ = it_.advertise("counting_image", 1, true);

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
    case 'n':
    case 'N':
      displayMode = MODEL;
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

  bool readInfoFromJson(const std::string &pathToJson, cv::Rect &rect, std::string &gtin)
  {
    std::ifstream ifs(pathToJson);
    rapidjson::IStreamWrapper isw(ifs);
    rapidjson::Document d;
    d.ParseStream(isw);

    if(d.IsObject() && d.HasMember("rect")) {
      rect.x =   d["rect"]["x"].GetInt();
      rect.y = d["rect"]["y"].GetInt();
      rect.height  = d["rect"]["h"].GetInt();
      rect.width = d["rect"]["w"].GetInt();
      gtin = std::to_string(d["dan"].GetInt64());
      return true;
    }
    else {
      outWarn(pathToJson << " is an empty file perhaps?");
      return false;
    }
  }

  cv::Mat getMaskForModelImage(const cv::Mat &img)
  {
    cv::Mat bin, canny, imgGrey;
    cv::cvtColor(img, imgGrey, CV_BGR2GRAY);
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
      if(area > 0.10 * imgGrey.cols * imgGrey.rows && hierarchy[idx][3] == -1) {
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
              pathToImageFile = it2->second["model_path"].as<std::string>();
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
    if(ctx.isParameterDefined("local_implementation"))
      ctx.extractValue("local_implementation",local_implementation_);
    if(!local_implementation_)
    {
     ac= new actionlib::SimpleActionClient<refills_msgs::ProductIdentificationAction>("/product_identification_server",
                                                                                true);
     outInfo("Waiting for [/product_identification_server]");
     ac->waitForServer();
     outInfo("Connected");
    } 
    return UIMA_ERR_NONE;
  }


  /**
   * @brief matchModelToFacing perform matching and getSimilarity score
   * @return
   */
  double matchModelToFacing(cv::Mat facingImg, std::vector<std::string> modelImages)
  {
//    cv::normalize(facingImg, facingImg);
    cv::GaussianBlur(facingImg, facingImg, cv::Size(9, 9), 0);

    cv::Mat facingHist = calcHistogram(facingImg, cv::Mat(), true);

    std::vector<cv::KeyPoint> facingKeypoints, modelKeypoints;
    std::vector<cv::Point2f> goodKp1, goodKp2, transformedKps;
    cv::Mat descriptors1, descriptors2;

    //keypoints for the facing
    detector->detectAndCompute(facingImg, cv::noArray(), facingKeypoints, descriptors1);
    outInfo("Detected " << facingKeypoints.size() << " keypoints in facing image");

    cv::Mat modelImage;
    cv::Mat modelHist;
    for(std::string &mi : modelImages) {
      outInfo("Comparing against mode image located at: " << mi);
      modelImage = cv::imread(mi, cv::IMREAD_ANYCOLOR);
      if(modelImage.type() == CV_8UC4 || modelImage.type() == CV_64FC4) {
        outInfo("uint with 4 uint");
        cv::Mat  channels[4];
        std::vector<cv::Mat> rgb;
        cv::split(modelImage, channels);
        rgb.push_back(channels[0]);
        rgb.push_back(channels[1]);
        rgb.push_back(channels[2]);
        cv::merge(rgb, modelImage);
        outInfo("merged");
      }
      model_ = modelImage.clone();
      //      cv::resize(modelImage, modelImage, cv::Size(0, 0), 0.75, 0.75, cv::INTER_AREA);
      cv::Mat modelMask = getMaskForModelImage(modelImage);
      modelHist = calcHistogram(modelImage, modelMask, true);
      cv::GaussianBlur(modelImage, modelImage, cv::Size(9, 9), 0);
      detector->detectAndCompute(modelImage, modelMask, modelKeypoints, descriptors2);
      break;
    }
    outInfo("Detected " << modelKeypoints.size() << " keypoints in model image");
    double histSimilarity = 0.0;
    if(!modelHist.empty() && !facingHist.empty())
      histSimilarity = 1 - cv::compareHist(facingHist, modelHist, CV_COMP_HELLINGER);

    cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);
    std::vector<cv::DMatch> matches;

    outInfo("Size of descriptors: " << descriptors1.size() << " D2: " << descriptors2.size());
    if(!descriptors1.empty() && !descriptors2.empty()) {
      matcher->match(descriptors1, descriptors2, matches);
      std::sort(matches.begin(), matches.end(), [](cv::DMatch a, cv::DMatch b) {
        return  a.distance < b.distance;
      });
    }
    if (matches.empty()) return 0.0;
    
    std::vector<char> mask(matches.size(), 0);
    for(int i = 0; i < matches.size() && matches[i].distance <= 60; ++i) {
      mask[i] = 1;
    }

    float avgDist = 0.0;
    uint8_t mi = 0;

    for(; mi < matches.size() && mi < 50; ++mi) {
      avgDist += (1 - static_cast<float>(matches[mi].distance / 256));
    }
    if (mi!=0) avgDist /= mi;
    mi = 0;
    for(; mi < matches.size() && matches[mi].distance <= 60   ; ++mi) {
      goodKp1.push_back(facingKeypoints[matches[mi].queryIdx].pt);
      goodKp2.push_back(modelKeypoints[matches[mi].trainIdx].pt);
    }

    cv::Mat homograpy;
    if(goodKp1.size() >= 4 && goodKp1.size() >= 4) {
      homograpy = cv::findHomography(goodKp1, goodKp2, CV_RANSAC);
      cv::perspectiveTransform(goodKp1, transformedKps, homograpy);
      float sum = 0.0;
      for(int j = 0; j < transformedKps.size(); ++j) {
        sum += std::sqrt((transformedKps[j].x - goodKp2[j].x) * (transformedKps[j].x - goodKp2[j].x)  + (transformedKps[j].y - goodKp2[j].y) * (transformedKps[j].y - goodKp2[j].y));
      }
      sum = sum / transformedKps.size() + 1;
      outInfo("Transform error:" << sum);
    }

    cv::drawMatches(facingImg, facingKeypoints, modelImage, modelKeypoints, matches, disp_, cv::Scalar::all(-1),
                    cv::Scalar::all(-1), mask, cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    outInfo("Histogram similarity (1 - Hellinger): " << histSimilarity);
    outInfo("Average dist of best 50s matches: " << avgDist);

    double measure = (avgDist + histSimilarity) / 2;
    outInfo("Final Similarity Measure is: " << measure);
    return avgDist;
  }

  cv::Mat calcHistogram(cv::Mat &img, cv::Mat mask = cv::Mat(), bool color = false)
  {
    cv::Mat histModel, histModelNormed;
    if(color) {
      cv::Mat imgHsv;
      cv::cvtColor(img, imgHsv, CV_BGR2HSV);
      int h_bins = 16;
      int s_bins = 8;
      int histSize[] = { h_bins, s_bins };
      float h_ranges[] = { 0, 256 };
      float s_ranges[] = { 0, 180 };
      const float *ranges[] = { h_ranges, s_ranges };
      int channels[] = { 0, 1};
      cv::calcHist(&imgHsv, 1, channels, mask, histModel, 2, histSize, ranges, true, false);
      cv::normalize(histModel, histModelNormed, 0, 1.0, cv::NORM_MINMAX, -1, cv::Mat());
      outInfo(histModel.size());
//      hist_.push_back(histImage);

      return histModelNormed;
    }
    else {
      cv::Mat imgGrey;
      cv::cvtColor(img, imgGrey, CV_BGR2GRAY);
      /// Establish the number of bins
      int histSize = 64;
      /// Set the ranges ( for B,G,R) )
      float range[] = { 0, 256 } ;
      const float *histRange = { range };


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
  }

  double callExternalAction(const cv::Mat& image, const cv::Rect& facing_rect, std::string gtin, std::string &resGtin)
  {
  //  actionlib::SimpleActionClient<refills_msgs::ProductIdentificationAction> ac("/product_identification_server",
  // true);
  //  outInfo("Waiting for action server to start.");
    // wait for the action server to start
  //  ac.waitForServer();  // will wait for infinite time
  //  outInfo("Action server started, sending goal.");
    // send a goal to the action

    refills_msgs::ProductIdentificationGoal goal;
    sensor_msgs::CompressedImage compr_image;
    compr_image.format = "jpeg";
    compr_image.header.stamp = ros::Time::now();
    cv::imencode(".jpg", image, compr_image.data);

    sensor_msgs::RegionOfInterest roi;
    roi.width = facing_rect.width;
    roi.height = facing_rect.height;
    roi.x_offset = facing_rect.x;
    roi.y_offset = facing_rect.y;

    goal.image = compr_image;
    goal.roi = roi;
    goal.goal_gtin  = gtin;
    ac->sendGoal(goal);

    // wait for the action to return
    bool finished_before_timeout = ac->waitForResult(ros::Duration(5.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac->getState();
      refills_msgs::ProductIdentificationResultConstPtr res = ac->getResult();
      resGtin = res->result_gtin;
      outInfo("Action finished: "<<state.toString());
      return res->confidence;
    }
    else return 0.0;
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
    hist_ = cv::Mat();
    cv::Mat facingImg;
    cv::Rect facingRect;
    std::string gTinOfFacing;

    std::string testFileName = "";

    std::vector<std::string> modelImages;
    if(!testMode_) {
      rs::SceneCas cas(tcas);
      rs::Scene scene = cas.getScene();
      
      cas.get(VIEW_COLOR_IMAGE, rgb);
      cas.get(VIEW_CAMERA_INFO, cam_info_);
      cas.get("display_image",disp_img_);

      std::vector<rs::ObjectHypothesis> hyps;
      scene.identifiables.filter(hyps);
      outInfo("Found " << hyps.size() << " hypotheses.");
      for(auto &h : hyps) {
        std::vector<rs::Detection> detections;
        h.annotations.filter(detections);
        if(detections.empty()) continue;

        rs::Detection &det = detections[0];
        //        det.confidence.set(0.77);

        if(det.source()  == "FacingDetection") {
          outInfo("Found a hypothesis with a facing detection");
          rs::ImageROI roi = h.rois();
          rs::conversion::from(roi.roi(), facingRect);
          outInfo("Rect of facing: " << facingRect);
          facingImg = rgb(facingRect);
          gTinOfFacing = det.name();
          outInfo("Gtin of Facing: " << gTinOfFacing);
          modelImages = gtinToImages_[gTinOfFacing];
          for(auto mI : modelImages)
          outInfo("Model Image fot Gtin found  at: " << mI);
          double score = 0.0;
          std::string actualGtin ="";
          if(local_implementation_)
            score = matchModelToFacing(facingImg, modelImages);
          else{
            score  = callExternalAction(rgb, facingRect, gTinOfFacing,actualGtin);
            outInfo("Intel detector says it is:" <<actualGtin);
            cv::Point text_loc, text_img_loc;


            int baseline=0; 
            cv::Size textSize = cv::getTextSize(actualGtin, cv::FONT_HERSHEY_COMPLEX, 1.0, 2, &baseline);
            cv::Mat textImage(textSize.height+10, textSize.width+10, disp_img_.type(), cv::Scalar(255,255, 255));
            text_loc.x = 5;
            text_loc.y = textSize.height+5;

            text_img_loc.x = std::max(0,facingRect.x - textSize.height-10);
            text_img_loc.y = std::max(0, facingRect.y);

            rs::Classification c = rs::create<rs::Classification>(tcas);
            c.classname.set(actualGtin);
            c.source.set("MisplacedItemDetection");
            h.annotations.append(c);

            if(actualGtin == gTinOfFacing){
                cv::putText(textImage,actualGtin, text_loc, cv::FONT_HERSHEY_COMPLEX, 1.0,cv::Scalar(0,255,0),2);
            }
            else{
                cv::putText(textImage,actualGtin,text_loc,cv::FONT_HERSHEY_COMPLEX, 1.0,cv::Scalar(0,0,255),2);
            }

            cv::Point2f src_center(textImage.cols/2.0F, textImage.rows/2.0F);
            cv::Mat rot_mat = cv::getRotationMatrix2D(src_center, 90 , 1.0);

            cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), textImage.size(), 90).boundingRect2f();
            // adjust transformation matrix
            rot_mat.at<double>(0,2) += bbox.width/2.0 - textImage.cols/2.0;
            rot_mat.at<double>(1,2) += bbox.height/2.0 - textImage.rows/2.0;

            cv::Mat text_rotated;
            cv::warpAffine(textImage, text_rotated, rot_mat, bbox.size());

            int diff = facingRect.height - text_rotated.rows;
            text_rotated.copyTo(disp_img_(cv::Rect(text_img_loc.x, text_img_loc.y + diff, text_rotated.cols, text_rotated.rows)));
          }
          det.confidence.set(score);
        }
      }
      //TOOD: use image transport
      cv_bridge::CvImage outImgMsgs;

      outImgMsgs.header = cam_info_.header;
      outImgMsgs.encoding = sensor_msgs::image_encodings::BGR8;
      outImgMsgs.image = disp_img_;
      image_pub_.publish(outImgMsgs.toImageMsg());


    }
    else {
      std::pair<std::string, std::string> &testFilePair = testFiles_[imgIdx_];
      testFileName = testFilePair.first;
      rgb = cv::imread(testFilePair.first);// cv::IMREAD_GRAYSCALE);
      if(!readInfoFromJson(testFilePair.second, facingRect, gTinOfFacing)) {
        imgIdx_++;
        return 0.0;
      }
      facingImg = rgb(facingRect);

      //      std::map<std::string, std::vector<std::string> >::iterator it = gtinToImages_.begin();
      //   readInfoFromJson   it = std::next(it, modelIdx_++);
      //      if(modelIdx_ == gtinToImages_.size()) {
      //        modelIdx_ = 0;
      //        imgIdx_++;
      //      }
      //      std::string modelGtin = it->first;
      //      modelImages = it->second;
      modelImages = gtinToImages_[gTinOfFacing];
      imgIdx_++;
      double  score = matchModelToFacing(facingImg, modelImages);
    }


    //    bfs::path savefolderPath(ros::package::getPath("rs_refills") + "/test_results/" + gTinOfFacing + "/");
    //    if(!bfs::exists(savefolderPath)) boost::filesystem::create_directory(savefolderPath);



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


    //    if(modelGtin == gTinOfFacing) {
    //      positiveMatch << sum << ",";
    //      outError("THIS IS THE REAL MATCH");
    //    }
    //    else {
    //      negativeMatch << sum << ",";
    //    }


    if(imgIdx_ == testFiles_.size() && testMode_) {
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
    case MODEL:
      disp = model_.clone();
      break;
    default:
      disp = disp_.clone();
      break;
    }
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(MisplacedItemDetection)
