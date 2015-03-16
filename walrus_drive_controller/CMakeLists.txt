cmake_minimum_required(VERSION 2.8.3)
project(walrus_drive_controller)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  nav_msgs
  realtime_tools
  tf
  urdf
  message_generation
  std_msgs
)

add_message_files(
  FILES
  TankDriveCommand.msg
  TankDriveCommandStamped.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  LIBRARIES walrus_drive_controller
  CATKIN_DEPENDS nav_msgs controller_interface realtime_tools tf urdf message_runtime std_msgs
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
add_dependencies(walrus_drive_controller ${PROJECT_NAME}_generate_messages_cpp)

#############
## Install ##
#############

install(TARGETS walrus_drive_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  walrus_drive_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

