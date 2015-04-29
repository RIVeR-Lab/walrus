cmake_minimum_required(VERSION 2.8.3)
project(position_effort_controller)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  std_msgs
  realtime_tools
  tf
  urdf
  control_toolbox
  control_msgs
  angles
  message_generation
)

add_message_files(
  FILES
  PositionEffortCommand.msg
  PositionEffortCommandStamped.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  LIBRARIES position_effort_controller
  CATKIN_DEPENDS std_msgs controller_interface realtime_tools tf urdf control_toolbox control_msgs angles message_runtime
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(position_effort_controller
  src/position_effort_controller.cpp
)

target_link_libraries(position_effort_controller
  ${catkin_LIBRARIES}
)
add_dependencies(position_effort_controller ${PROJECT_NAME}_generate_messages_cpp)

#############
## Install ##
#############

install(TARGETS position_effort_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  position_effort_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

