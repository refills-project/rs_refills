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

#include <robosherlock/flowcontrol/RSProcessManager.h>


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
  RSRefillsProcessManager(std::string aae_file, bool usevis, bool wait, ros::NodeHandle nh): RSProcessManager(aae_file, usevis, rs::KnowledgeEngine::KnowledgeEngineType::SWI_PROLOG)
  {
    this->setWaitForService(wait); 
  }
  virtual ~RSRefillsProcessManager()
  {

  }

  bool handleQuery(std::string &req, std::vector<std::string> &res)
  {
    outInfo("Handling Query for Refills stuff");
    outInfo("JSON Reuqest: " << req);
    query_interface_->parseQuery(req);
    std::vector<std::string> newPipelineOrder;
    std::vector<std::vector<std::string>> newPipelineOrders;
    QueryInterface::QueryType queryType = query_interface_->processQuery(newPipelineOrders);
    for(auto p : newPipelineOrder) 
      outInfo(p);

    //these are hacks that should be handled by integration of these components in the pipeline planning process
    rapidjson::Document& query = query_interface_->getQueryDocument();

    if(newPipelineOrder.empty() && queryType == QueryInterface::QueryType::SCAN) {
      rapidjson::Value &val = query["scan"];
      if(val.HasMember("type")) {
        std::string  type = val["type"].GetString();

        //TODO: this whole part should move to knowrob pipeline planni
        if(type == "shelf") {
          newPipelineOrder.push_back("CollectionReader");
          newPipelineOrder.push_back("ImagePreprocessor");
          newPipelineOrder.push_back("NormalEstimator");
          newPipelineOrder.push_back("ShelfDetector");
         // newPipelineOrder.push_back("StorageWriter");
        }
        if(type == "facing") {
          newPipelineOrder.push_back("CollectionReader");
          newPipelineOrder.push_back("ImagePreprocessor");
          newPipelineOrder.push_back("NormalEstimator");
          newPipelineOrder.push_back("SeparatorScanner");
          newPipelineOrder.push_back("BarcodeScanner");
         // newPipelineOrder.push_back("StorageWriter");
        }
        if(type == "barcode") {
          newPipelineOrder.push_back("CollectionReader");
          newPipelineOrder.push_back("ImagePreprocessor");
          newPipelineOrder.push_back("NormalEstimator");
          newPipelineOrder.push_back("BarcodeScanner");
         // newPipelineOrder.push_back("StorageWriter");
        }
        if(type == "separator") {
          newPipelineOrder.push_back("CollectionReader");
          newPipelineOrder.push_back("ImagePreprocessor");
          newPipelineOrder.push_back("NormalEstimator");
          newPipelineOrder.push_back("SeparatorScanner");
          //newPipelineOrder.push_back("StorageWriter");
        }
      }
    }
    if(queryType == QueryInterface::QueryType::DETECT) {
      newPipelineOrder.clear();
      newPipelineOrder.push_back("CollectionReader");
      newPipelineOrder.push_back("ImagePreprocessor");
      newPipelineOrder.push_back("NormalEstimator");
      newPipelineOrder.push_back("ProductCounter");
      newPipelineOrder.push_back("MisplacedItemDetection");
      newPipelineOrder.push_back("ImageSaver");
     // newPipelineOrder.push_back("StorageWriter");
    }

    {
      std::lock_guard<std::mutex> lock(processing_mutex_);
      if(queryType == QueryInterface::QueryType::SCAN) 
      {
        rapidjson::Value& val = query["scan"];
        if (val.HasMember("command"))
        {
           std::string command = val["command"].GetString();
           engine_->resetCas();
           rs::Query query = rs::create<rs::Query>(*engine_->getCas());
           query.query.set(req);
           rs::SceneCas sceneCas(*engine_->getCas());
           outInfo("Setting query in CAS");
           sceneCas.setFS("QUERY", query);
           if (command == "start")
           {
             engine_->setContinuousPipelineOrder(newPipelineOrder);
             engine_->setPipelineOrdering(newPipelineOrder);
             engine_->processOnce();
             wait_for_service_call_ = false;
             return true;
           }
           else if (command == "stop")
           {
             wait_for_service_call_ = true;
             engine_->setPipelineOrdering(newPipelineOrder);
             engine_->processOnce(res, req);
             rs::ObjectDesignatorFactory dw(engine_->getCas(), rs::ObjectDesignatorFactory::Mode::CLUSTER);
             dw.getObjectDesignators(res);
             return true;
           }
         }
      }
      else if(queryType == QueryInterface::QueryType::DETECT)
      {
        engine_->resetCas();
        rs::Query query = rs::create<rs::Query>(*engine_->getCas());
        query.query.set(req);
        rs::SceneCas sceneCas(*engine_->getCas());
        sceneCas.setFS("QUERY", query);
        engine_->setPipelineOrdering(newPipelineOrder);
        engine_->processOnce(res, req);
        outInfo("Converting to Json");
        rs::ObjectDesignatorFactory dw(engine_->getCas(), rs::ObjectDesignatorFactory::Mode::CLUSTER);
        dw.getObjectDesignators(res);
        return true;
       }
      else
      {
        outError("Malformed query: The refills scenario only handles Scanning commands(for now)");
        return false;
      }
    }//end of lock
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

  ros::service::waitForService("/rosprolog/query");
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
    RSRefillsProcessManager manager(analysisEngineFile, useVisualizer, waitForServiceCall, nh);
    manager.setUseIdentityResolution(false);
    manager.run();
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
