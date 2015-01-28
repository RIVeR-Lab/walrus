cmake_minimum_required(VERSION 2.8.3)

include(${TEENSYDUINO})

include_directories(${ROS_LIB_DIR})

generate_arduino_firmware(diagnostic_board_firmware
  SRCS main.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD teensypp2
  PORT /dev/ttyACM0
)