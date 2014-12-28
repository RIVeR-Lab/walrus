cmake_minimum_required(VERSION 2.8.3)
project(walrus_web)

find_package(catkin REQUIRED)

catkin_package()

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
