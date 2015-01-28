cmake_minimum_required(VERSION 2.8.3)
project(diagnostic_board_firmware)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  rosserial_arduino
  rosserial_client
  walrus_firmware_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS std_msgs rosserial_arduino rosserial_client walrus_firmware_msgs
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(ARDUINO_SDK_PATH ../../teensyduinoSDK/arduino-1.0.6)

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

#rosserial_configure_client(
 # DIRECTORY firmware
 # TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
#)


  set(DTOOLCHAIN_FILE -DCMAKE_TOOLCHAIN_FILE=${ROSSERIAL_ARDUINO_TOOLCHAIN})
  # Create a build tree directory for configuring the client's CMake project.
  file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/firmware)
  add_custom_target(${PROJECT_NAME}_firmware
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/firmware
    COMMAND ${CMAKE_COMMAND} ${PROJECT_SOURCE_DIR}/firmware
      -DROS_LIB_DIR=${${PROJECT_NAME}_ROS_LIB_DIR}
      -DARDUINO_SDK_PATH=${CMAKE_SOURCE_DIR}/../../teensyduinoSDK/arduino-1.0.6
      -DEXECUTABLE_OUTPUT_PATH=${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}
      ${DTOOLCHAIN_FILE}
  )
  add_dependencies(${PROJECT_NAME}_firmware ${PROJECT_NAME}_ros_lib)

rosserial_add_client_target(firmware diagnostic_board_firmware ALL)
rosserial_add_client_target(firmware diagnostic_board_firmware-upload)
