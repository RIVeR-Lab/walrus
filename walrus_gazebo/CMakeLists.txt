cmake_minimum_required(VERSION 2.8.3)
project(walrus_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_ros_control
  pluginlib
  roscpp
  transmission_interface
  urdf
)

find_package(Gazebo REQUIRED)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(walrus_hw_sim
  src/walrus_hw_sim.cpp
)
target_link_libraries(walrus_hw_sim
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS walrus_hw_sim
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  walrus_hw_sim_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
