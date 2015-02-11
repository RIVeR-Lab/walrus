cmake_minimum_required(VERSION 2.8.3)
project(walrus_bootloader)

find_package(catkin REQUIRED)

catkin_package()

###########
## Build ##
###########


add_custom_command(OUTPUT BootloaderCDC_src
  COMMAND ${CMAKE_COMMAND} -E copy_directory
  ${CMAKE_SOURCE_DIR}/src ${CMAKE_BINARY_DIR}/src
  COMMENT "Copying bootloader source to build directory"
)

add_custom_target(BootloaderCDC ALL
  COMMAND make
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/src
  COMMENT "Building bootloader"
  DEPENDS BootloaderCDC_src
)

add_custom_command(TARGET BootloaderCDC POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy
  ${CMAKE_BINARY_DIR}/src/BootloaderCDC.hex ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/BootloaderCDC.hex
  COMMENT "Copying bootloader hex to output directory"
)



#############
## Install ##
#############

install(PROGRAMS
  upload.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/BootloaderCDC.hex
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
