cmake_minimum_required(VERSION 2.8.3)
project(walrus_base_hw)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  controller_manager_msgs
  hardware_interface
  roscpp
  transmission_interface
  epos_hardware
)

find_package(Boost COMPONENTS chrono)

if(Boost_CHRONO_FOUND)
  add_definitions(-DHAVE_BOOST_CHRONO=1)
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES walrus_robot_base
  CATKIN_DEPENDS controller_manager_msgs controller_manager hardware_interface roscpp transmission_interface
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)

add_library(walrus_robot_base src/walrus_robot_base.cpp)
target_link_libraries(walrus_robot_base
  ${catkin_LIBRARIES}
)

add_executable(epos_hw_node src/epos_hw_node.cpp)
target_link_libraries(epos_hw_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)


#############
## Install ##
#############

install(TARGETS walrus_robot_base epos_hw_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
