cmake_minimum_required(VERSION 2.8.3)
project(walrus_boomboard_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  walrus_firmware_msgs
  hardware_interface
  transmission_interface
  controller_manager
  roscpp
  diagnostic_updater
  walrus_base_hw
  rosserial_server
)

###################################
## catkin specific configuration ##
###################################
catkin_package(CATKIN_DEPENDS walrus_firmware_msgs)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(walrus_boomboard_node
  src/${PROJECT_NAME}/walrus_boomboard_robot.cpp
  src/${PROJECT_NAME}/walrus_boomboard_node.cpp
)

target_link_libraries(walrus_boomboard_node
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS walrus_boomboard_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
