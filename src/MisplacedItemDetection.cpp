#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>
//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>

//yaml
#include <yaml-cpp/yaml.h>

#include <ros/package.h>

using namespace uima;


class MisplacedItemDetection : public Annotator
{
private:
  float test_param;

  std::map<std::string, std::vector<std::string>> gtinToImages;
  YAML::Node yamlModelsConfig;
public:

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
        //        outInfo(gtin);
        if(value.Type() == YAML::NodeType::Map) {
          for(YAML::const_iterator it2 = value.begin(); it2 != value.end(); ++it2) {
            //            outInfo(it2->first.as<std::string>());
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

    }
    for(auto m : gtinToImages) {
      outInfo(m.first);
      for(auto f : m.second)
        outInfo(f);
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
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();
    std::vector<rs::ObjectHypothesis> hyps;
    scene.identifiables.filter(hyps);
    return UIMA_ERR_NONE;
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(MisplacedItemDetection)
