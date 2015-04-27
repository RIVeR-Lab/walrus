cmake_minimum_required(VERSION 2.8.3)
project(walrus_de2i_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs
)

###########
## Build ##
###########
include_directories(${catkin_INCLUDE_DIRS})

add_executable(walrus_de2i_driver_node src/walrus_de2i_driver_node.cpp)
target_link_libraries(walrus_de2i_driver_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS walrus_de2i_driver_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

