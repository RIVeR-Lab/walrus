cmake_minimum_required(VERSION 2.8.3)
project(walrus_joystick_controller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
)

catkin_package()

###########
## Build ##
###########
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(joystick_controller_node src/joystick_controller_node.cpp src/joystick_controller.cpp)

target_link_libraries(joystick_controller_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS  joystick_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
