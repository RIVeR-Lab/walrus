cmake_minimum_required(VERSION 2.8.3)
project(rosserial_teensyduino)

find_package(catkin REQUIRED COMPONENTS
  rosserial_arduino
)


catkin_package(
  CATKIN_DEPENDS rosserial_arduino
  CFG_EXTRAS rosserial_teensyduino-extras.cmake
)

install(DIRECTORY teensyduino_sdk
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES cmake/Teensyduino.cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
)
