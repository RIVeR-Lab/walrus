cmake_minimum_required(VERSION 2.8.3)

include(${TEENSYDUINO})

file(READ SoftI2CMaster.h.in SOFT_I2C_MASTER_TEMPLATE)

foreach(COUNT RANGE 7)
  string(REPLACE "XXX" "${COUNT}" SOFT_I2C_MASTER_VERSION "${SOFT_I2C_MASTER_TEMPLATE}")
  file(WRITE "${CMAKE_BINARY_DIR}/include/SoftI2CMaster${COUNT}.h" "${SOFT_I2C_MASTER_VERSION}")
  message("Generated SoftI2CMaster${COUNT}.h")
endforeach(COUNT)

include_directories(${ROS_LIB_DIR})

generate_arduino_firmware(main_board_firmware
  SRCS main.cpp ${ROS_LIB_DIR}/time.cpp SmartBatt.cpp i2c_busses.cpp
  BOARD teensypp2
  PORT /dev/ttyACM0
)