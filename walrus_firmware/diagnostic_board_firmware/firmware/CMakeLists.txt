cmake_minimum_required(VERSION 2.8.3)

register_hardware_platform(${CMAKE_SOURCE_DIR}/../../../teensyduinoSDK/arduino-1.0.6/hardware/teensy)
message(${CMAKE_SOURCE_DIR}/../../../teensyduinoSDK/arduino-1.0.6/hardware/teensy)


add_definitions(-DLAYOUT_US_ENGLISH)
add_definitions(-DUSB_SERIAL)
add_definitions(-D__AVR_AT90USB1286__)
set(teensypp2.build.f_cpu 16000000L)

include_directories(${ROS_LIB_DIR})

generate_arduino_firmware(diagnostic_board_firmware
  SRCS main.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD teensypp2
  PORT /dev/ttyACM0
)