cmake_minimum_required(VERSION 2.8.3)
project(multi_usb_cam)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  usb_cam
  sensor_msgs
  camera_info_manager
)

catkin_package(
  CATKIN_DEPENDS image_transport roscpp usb_cam sensor_msgs camera_info_manager
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(multi_usb_cam_node src/multi_usb_cam_node.cpp src/multi_usb_cam.cpp)
target_link_libraries(multi_usb_cam_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS multi_usb_cam_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
