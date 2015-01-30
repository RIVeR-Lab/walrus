cmake_minimum_required(VERSION 2.8.3)
project(main_board_firmware)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  rosserial_arduino
  rosserial_teensyduino
  rosserial_client
  walrus_firmware_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS std_msgs rosserial_arduino rosserial_teensyduino rosserial_client walrus_firmware_msgs
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_teensyduino_client(
 DIRECTORY firmware

rosserial_add_client_target(firmware main_board_firmware ALL)
rosserial_add_client_target(firmware main_board_firmware-upload)
