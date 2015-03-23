cmake_minimum_required(VERSION 2.8.3)
project(walrus_interface_controller)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  roscpp
)


add_message_files(
  FILES
  InterfaceControllerState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(interface_controller_node src/interface_controller_node.cpp)
add_dependencies(interface_controller_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(interface_controller_node
  ${catkin_LIBRARIES}
)

add_executable(interface_manager_node src/interface_manager_node.cpp)
add_dependencies(interface_manager_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(interface_manager_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS interface_controller_node interface_manager_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
