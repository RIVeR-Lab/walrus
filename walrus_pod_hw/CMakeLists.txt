cmake_minimum_required(VERSION 2.8.3)
project(walrus_pod_hw)

find_package(catkin REQUIRED COMPONENTS
  walrus_firmware_msgs
  roscpp
  diagnostic_updater
  walrus_base_hw
  hardware_interface
  transmission_interface
  controller_manager
  rosserial_server
  roboteq_driver
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS walrus_firmware_msgs diagnostic_updater walrus_base_hw hardware_interface transmission_interface controller_manager rosserial_server roboteq_driver
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(walrus_pod_hw_node
   src/walrus_pod_hw.cpp
   src/walrus_pod_hw_node.cpp
)
target_link_libraries(walrus_pod_hw_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS walrus_pod_hw_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
