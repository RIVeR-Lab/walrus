cmake_minimum_required(VERSION 2.8.3)
project(walrus_drive_controller)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  nav_msgs
  realtime_tools
  tf
  urdf
)

catkin_package(
  LIBRARIES walrus_drive_controller
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(walrus_drive_controller
  src/walrus_drive_controller.cpp
  src/speed_limiter.cpp
  src/odometry.cpp
)

target_link_libraries(walrus_drive_controller
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS walrus_drive_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
