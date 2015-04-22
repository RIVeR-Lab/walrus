cmake_minimum_required(VERSION 2.8.3)

include(${TEENSYDUINO})

file(READ SoftI2CMaster.h.in SOFT_I2C_MASTER_TEMPLATE)

foreach(COUNT RANGE 7)
  string(REPLACE "XXX" "${COUNT}" SOFT_I2C_MASTER_VERSION "${SOFT_I2C_MASTER_TEMPLATE}")
  file(WRITE "${CMAKE_BINARY_DIR}/include/SoftI2CMaster${COUNT}.h" "${SOFT_I2C_MASTER_VERSION}")
  message("Generated SoftI2CMaster${COUNT}.h")
endforeach(COUNT)

include_directories(${ROS_LIB_DIR} "${CMAKE_BINARY_DIR}/include/")

add_definitions(-DSTR_MANUFACTURER=L"WALRUS Rover MQP Team")
add_definitions(-DSTR_PRODUCT=L"WALRUS Main Control Board")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -gdwarf-2")

generate_arduino_firmware(main_board_firmware
  SRCS main.cpp ${ROS_LIB_DIR}/time.cpp SmartBatt.cpp i2c_buses.cpp SoftwareSerial.cpp
  BOARD teensypp2
  PORT /dev/ttyACM0
)
