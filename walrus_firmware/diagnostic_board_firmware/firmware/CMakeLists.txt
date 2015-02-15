cmake_minimum_required(VERSION 2.8.3)

include(${TEENSYDUINO})

include_directories(${ROS_LIB_DIR})

add_definitions(-DSTR_MANUFACTURER=L"WALRUS Rover MQP Team")
add_definitions(-DSTR_PRODUCT=L"WALRUS Diagnostic Board")

generate_arduino_firmware(diagnostic_board_firmware
  SRCS main.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD teensypp2
  PORT /dev/ttyACM0
)
