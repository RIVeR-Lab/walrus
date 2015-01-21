cmake_minimum_required(VERSION 2.8.3)
project(walrus_web)

find_package(catkin REQUIRED)

execute_process(
  COMMAND ./download_src_deps.sh
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/
)

catkin_package()

#############
##  Build  ##
#############

add_custom_target(build ALL COMMAND grunt WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

#############
## Install ##
#############

install(PROGRAMS
  scripts/webserver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY web
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
