cmake_minimum_required(VERSION 2.8.3)
project(walrus_web_interface_controller)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

#############
## Install ##
#############

install(FILES
  launch/web_interface.launch
)
