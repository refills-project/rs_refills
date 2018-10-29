/*------------------------------------------------------------------------

 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.

 --------------------------------------------------------------------------

 Test driver that reads text files or XCASs or XMIs and calls the annotator

 -------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <stdio.h>
#include <errno.h>
#include <sys/stat.h>
#include <chrono>
#include <condition_variable>
#include <sstream>

#include <uima/api.hpp>
#include "uima/internal_aggregate_engine.hpp"
#include "uima/annotator_mgr.hpp"

#include <rs/flowcontrol/RSProcessManager.h>

#include <ros/ros.h>
#include <ros/package.h>

#undef OUT_LEVEL
#define OUT_LEVEL OUT_LEVEL_DEBUG


/**
 * Error output if program is called with wrong parameter.
 */

class RSRefillsProcessManager: public RSProcessManager
{
public:
  RSRefillsProcessManager(bool usevis, bool wait, ros::NodeHandle nh): RSProcessManager(usevis, wait, nh, ".")
  {

  }
  virtual ~RSRefillsProcessManager()
  {

  }

  bool handleQuery(std::string &req, std::vector<std::string> &res)
  {
    outInfo("Handling Query for Refills stuff");
    outInfo("JSON Reuqest: " << req);
    queryInterface->parseQuery(req);
    std::vector<std::string> newPipelineOrder;
    QueryInterface::QueryType queryType = queryInterface->processQuery(newPipelineOrder);
    for(auto p : newPipelineOrder) outInfo(p);
    //these are hacks that should be handled by integration of these components in the pipeline planning process
    if(newPipelineOrder.empty() && queryType == QueryInterface::QueryType::SCAN) {
      rapidjson::Value &val = queryInterface->query["scan"];
      if(val.HasMember("type")) {
        std::string  type = val["type"].GetString();

        //TODO: this whole part should move to knowrob pipeline planni
        if(type == "shelf") {
          newPipelineOrder.push_back("CollectionReader");
          newPipelineOrder.push_back("ImagePreprocessor");
          newPipelineOrder.push_back("NormalEstimator");
          newPipelineOrder.push_back("ShelfDetector");
        }
        if(type == "facing") {
          newPipelineOrder.push_back("CollectionReader");
          newPipelineOrder.push_back("ImagePreprocessor");
          newPipelineOrder.push_back("NormalEstimator");
          newPipelineOrder.push_back("SeparatorScanner");
          newPipelineOrder.push_back("BarcodeScanner");
        }
        if(type == "barcode") {
          newPipelineOrder.push_back("CollectionReader");
          newPipelineOrder.push_back("ImagePreprocessor");
          newPipelineOrder.push_back("NormalEstimator");
          newPipelineOrder.push_back("BarcodeScanner");
        }
        if(type == "separator") {
          newPipelineOrder.push_back("CollectionReader");
          newPipelineOrder.push_back("ImagePreprocessor");
          newPipelineOrder.push_back("NormalEstimator");
          newPipelineOrder.push_back("SeparatorScanner");
        }
      }
    }
    if(queryType == QueryInterface::QueryType::DETECT) {
      newPipelineOrder.clear();
      newPipelineOrder.push_back("CollectionReader");
      newPipelineOrder.push_back("ImagePreprocessor");
      newPipelineOrder.push_back("NormalEstimator");
      newPipelineOrder.push_back("ProductCounter");
    }

    {
      std::lock_guard<std::mutex> lock(processing_mutex_);
      if(queryType == QueryInterface::QueryType::SCAN) {
        rapidjson::Value &val = queryInterface->query["scan"];
        if(val.HasMember("command")) {
          std::string  command = val["command"].GetString();
          if(command == "start") {
            engine_.setQuery(req);
            engine_.changeLowLevelPipeline(newPipelineOrder);
            waitForServiceCall_ = false;
            return true;
          }
          else if(command == "stop") {
            waitForServiceCall_ = true;
            engine_.setNextPipeline(newPipelineOrder);
            engine_.applyNextPipeline();
            engine_.process(res, req);
            return true;
          }
        }
      }
      else if(queryType == QueryInterface::QueryType::DETECT) {
        engine_.setQuery(req);
        engine_.setNextPipeline(newPipelineOrder);
        engine_.applyNextPipeline();
        engine_.process(res, req);
        return true;
      }
      else {
        outError("Malformed query: The refills scenario only handles Scanning commands(for now)");
        processing_mutex_.unlock();
        return false;
      }
    }
    return false;
  }
};

void help()
{
  std::cout << "Usage : rosrun rs_queryanswering run [options] [analysis_engine]" << std::endl
            << "Options : " << std::endl
            << "              _ae : = engine1   analysis_engine to run" << std::endl
            << "              _vis : = true | false     use robosherlock visualization" << std::endl
            << std::endl;
}

/* ----------------------------------------------------------------------- */
/*       Main                                                              */
/* ----------------------------------------------------------------------- */

int main(int argc, char *argv[])
{
  if(argc < 2) {
    help();
    return 1;
  }

  ros::init(argc, argv, std::string("RoboSherlock"));
  ros::NodeHandle nh("~");

  std::string analysisEnginesName, analysisEngineFile;
  bool useVisualizer, waitForServiceCall = true;

  nh.param("ae", analysisEnginesName, std::string(""));
  nh.param("vis", useVisualizer, false);

  ros::service::waitForService("/json_prolog/simple_query");
  rs::common::getAEPaths(analysisEnginesName, analysisEngineFile);

  if(analysisEngineFile.empty()) {
    outError("analysis   engine \"" << analysisEngineFile << "\" not found.");
    return -1;
  }
  else {
    outInfo(analysisEngineFile);
  }

  std::string configFile = ros::package::getPath("rs_refills") + "/config/config_refills.yaml";

  try {
    RSRefillsProcessManager manager(useVisualizer, waitForServiceCall, nh);
    manager.setUseIdentityResolution(false);

    manager.pause();
    manager.init(analysisEngineFile, configFile, false, false);
    manager.run();
    manager.stop();
  }
  catch(const rs::Exception &e) {
    outError("Exception: " << std::endl << e.what());
    return -1;
  }
  catch(const uima::Exception &e) {
    outError("Exception: " << std::endl << e);
    return -1;
  }
  catch(const std::exception &e) {
    outError("Exception: " << std::endl << e.what());
    return -1;
  }
  catch(...) {
    outError("Unknown exception!");
    return -1;
  }
  return 0;
}
