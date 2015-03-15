cmake_minimum_required(VERSION 2.8.3)
project(walrus_base_hw)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  controller_manager_msgs
  hardware_interface
  roscpp
  transmission_interface
  epos_hardware
  rosserial_server
  walrus_mainboard_driver
  walrus_boomboard_driver
)

find_package(Boost COMPONENTS chrono)

if(Boost_CHRONO_FOUND)
  add_definitions(-DHAVE_BOOST_CHRONO=1)
endif()

catkin_package(CATKIN_DEPENDS controller_manager_msgs)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)

add_library(walrus_base_robot src/walrus_base_robot.cpp)
target_link_libraries(walrus_base_robot
  ${catkin_LIBRARIES}
)


add_executable(walrus_base_hw_node src/walrus_base_hw_node.cpp)
target_link_libraries(walrus_base_hw_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  walrus_base_robot
)

add_executable(walrus_base_epos_hw_node src/walrus_base_epos_hw_node.cpp)
target_link_libraries(walrus_base_epos_hw_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  walrus_base_robot
)


#############
## Install ##
#############

install(TARGETS walrus_base_robot walrus_base_hw_node walrus_base_epos_hw_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
