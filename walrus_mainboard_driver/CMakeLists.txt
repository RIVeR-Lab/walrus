cmake_minimum_required(VERSION 2.8.3)
project(walrus_mainboard_driver)

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
  std_msgs
  walrus_base_hw
  rosserial_server
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   CATKIN_DEPENDS walrus_firmware_msgs std_msgs
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(walrus_mainboard_node
   src/${PROJECT_NAME}/walrus_mainboard_robot.cpp
   src/${PROJECT_NAME}/walrus_mainboard_node.cpp
)

target_link_libraries(walrus_mainboard_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS walrus_mainboard_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
