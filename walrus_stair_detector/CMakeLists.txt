cmake_minimum_required(VERSION 2.8.3)
project(walrus_stair_detector)

option(walrus_stair_detector_ENABLE_TIMING "Enable timing" ON)
option(walrus_stair_detector_ENABLE_OMP "Enable OpenMP" ON)
option(walrus_stair_detector_ENABLE_VISUALIZATION "Enable visualization" OFF)
option(walrus_stair_detector_ENABLE_RANSAC_DEBUG "Enable RANSAC debug output" OFF)
option(walrus_stair_detector_ENABLE_DEBUG "Enable general debug output" OFF)


find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  pcl_conversions
  message_generation
  std_msgs
  geometry_msgs
)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Boost COMPONENTS timer)
include_directories(${Boost_INCLUDE_DIRS})


################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  Stair.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES walrus_stair_detector walrus_stair_detector_ros
  CATKIN_DEPENDS pcl_ros roscpp pcl_conversions message_runtime std_msgs geometry_msgs
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wno-sign-compare")

if(walrus_stair_detector_ENABLE_OMP)
  add_definitions(-DPCL_OMP=1)
endif()

if(walrus_stair_detector_ENABLE_VISUALIZATION)
  add_definitions(-DVISUALIZE=1 -DVISUALIZE_POINT_CLOUD=1 -DVISUALIZE_DETECTED_PLANES=0 -DVISUALIZE_DETECTED_PLANE_NORMALS=0 -DVISUALIZE_STAIR_AXES=0 -DVISUALIZE_STAIR_MEASUREMENTS=0 -DVISUALIZE_RISERS=0 -DVISUALIZE_STAIRS=1)
endif()

if(walrus_stair_detector_ENABLE_RANSAC_DEBUG)
  add_definitions(-DDEBUG_RANSAC=1)
endif()

if(walrus_stair_detector_ENABLE_DEBUG)
  add_definitions(-DDEBUG=1)
endif()

if(walrus_stair_detector_ENABLE_TIMING AND Boost_TIMER_FOUND)
  add_definitions(-DDEBUG_TIMING=1)
endif()



include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_library(walrus_stair_detector src/walrus_stair_detector.cpp)
target_link_libraries(walrus_stair_detector
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_library(walrus_stair_detector_ros src/walrus_stair_detector_ros.cpp)
target_link_libraries(walrus_stair_detector_ros
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  walrus_stair_detector
)
add_dependencies(walrus_stair_detector_ros ${PROJECT_NAME}_generate_messages_cpp)


add_executable(walrus_stair_detector_node src/walrus_stair_detector_node.cpp)
target_link_libraries(walrus_stair_detector_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  walrus_stair_detector_ros
)

add_executable(walrus_stair_detector_model_builder src/walrus_stair_detector_model_builder.cpp)
target_link_libraries(walrus_stair_detector_model_builder
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  walrus_stair_detector
)


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS walrus_stair_detector walrus_stair_detector_ros walrus_stair_detector_node walrus_stair_detector_model_builder
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(PkgConfig REQUIRED)
  pkg_search_module(YAMLCPP REQUIRED yaml-cpp)
  include_directories(${YAMLCPP_INCLUDE_DIRS})

  find_package(walrus_testdata REQUIRED)

  catkin_add_gtest(stair_detector_test test/stair_detector_test.cpp)
  target_link_libraries(stair_detector_test walrus_stair_detector ${YAMLCPP_LIBRARIES} ${catkin_LIBRARIES})
endif()
