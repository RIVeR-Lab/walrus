cmake_minimum_required(VERSION 2.8.3)
project(calf_gazebo)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  gazebo_ros
  gazebo_ros_control
  joint_limits_interface
  urdf
  pluginlib
  roscpp
  transmission_interface
)


find_package(Gazebo REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES calf_gazebo
  CATKIN_DEPENDS controller_manager gazebo_ros gazebo_ros_control joint_limits_interface urdf pluginlib roscpp transmission_interface
  DEPENDS Gazebo
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)


add_library(calf_hw_sim src/calf_hw_sim.cpp)
target_link_libraries(calf_hw_sim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS calf_gazebo calf_gazebo_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

install(FILES
  calf_hw_sim_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_calf_gazebo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
