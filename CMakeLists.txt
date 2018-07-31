cmake_minimum_required(VERSION 2.8.3)
project(rs_refills)
find_package(catkin REQUIRED robosherlock refills_msgs json_prolog) 
find_package(PCL 1.8 REQUIRED)

#need to add this so RSProcessManager from RoboSherlock has QueryInterface
add_definitions(-DWITH_JSON_PROLOG)

################################################################################
## Constants for project                                                      ##
################################################################################
set(NAMESPACE rs_refills)
set(TYPESYSTEM_CPP_PATH ${PROJECT_SOURCE_DIR}/include/rs_refills/types)
set(TYPESYSTEM_XML_PATH ${PROJECT_SOURCE_DIR}/descriptors/typesystem)
set(ANNOTATOR_PATH      ${PROJECT_SOURCE_DIR}/descriptors/annotators)
set(ENGINE_PATH         ${PROJECT_SOURCE_DIR}/descriptors/analysis_engines)
set(CMAKE_MODULE_PATH   ${PROJECT_SOURCE_DIR}/cmake)
################################################################################
## Configure project configuration files                                      ##
################################################################################
configure_file(${RS_PROJECT_CONFIG} ${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}_config.cmake @ONLY)
################################################################################
## Find all include directories                                               ##
################################################################################
find_include_dirs(RS_INCLUDE_DIRS_LIST)
catkin_package(
   CFG_EXTRAS ${PROJECT_NAME}_config.cmake
   )
################################################################################
## Package dependencies                                                       ##
################################################################################
#Add required catkin packages here
#find_package(catkin REQUIRED COMPONENTS xxx)
#Add required system packages here
#find_package(xxx)
################################################################################
## Build                                                                      ##
################################################################################
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        ${RS_INCLUDE_DIRS_LIST}
        ${catkin_INCLUDE_DIRS}
        )
################################################################################
## Update analysis engines, typesystem and include all relevant files         ##
################################################################################
##Update xml list of annotators inside analysis engines
update_analysis_engines(robosherlock)
## generate classes from the typesystem xml files
generate_type_system(robosherlock)
#find all relevant files
find_additional_files()
################################################################################

rs_add_library(rs_shelfDetector src/ShelfDetector.cpp)
target_link_libraries(rs_shelfDetector ${PCL_LIBRARIES} ${catkin_LIBRARIES})

rs_add_library(rs_productCounter src/ProductCounter.cpp)
target_link_libraries(rs_productCounter ${catkin_LIBRARIES})

rs_add_library(rs_separatorScanner src/SeparatorScanner.cpp)
target_link_libraries(rs_separatorScanner ${catkin_LIBRARIES})

rs_add_library(rs_barcodeScanner src/BarcodeScanner.cpp)
target_link_libraries(rs_barcodeScanner ${catkin_LIBRARIES})

rs_add_executable(processing_engine src/run.cpp)
target_link_libraries(processing_engine ${catkin_LIBRARIES})
