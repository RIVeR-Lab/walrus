cmake_minimum_required(VERSION 2.8.3)

@[if DEVELSPACE]@
set(TEENSYDUINO_SDK_PATH "@(CMAKE_CURRENT_SOURCE_DIR)/teensyduino_sdk/arduino-1.0.6")
set(TEENSYDUINO "@(CMAKE_CURRENT_SOURCE_DIR)/cmake/Teensyduino.cmake")
@[else]@
set(TEENSYDUINO_SDK_PATH "${rosserial_teensyduino_DIR}/teensyduino_sdk/arduino-1.0.6")
set(TEENSYDUINO "${rosserial_teensyduino_DIR}/cmake/Teensyduino.cmake")
@[end if]@

function(rosserial_configure_teensyduino_client)
  cmake_parse_arguments(client "" "DIRECTORY" "" ${ARGN})
  if(NOT client_DIRECTORY)
    message(SEND_ERROR "rosserial_client_add_client called without DIRECTORY argument.")
  endif()

  set(DTOOLCHAIN_FILE -DCMAKE_TOOLCHAIN_FILE=${ROSSERIAL_ARDUINO_TOOLCHAIN})

  # Create a build tree directory for configuring the client's CMake project.
  file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/${client_DIRECTORY})
  add_custom_target(${PROJECT_NAME}_${client_DIRECTORY}
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/${client_DIRECTORY}
    COMMAND ${CMAKE_COMMAND} ${PROJECT_SOURCE_DIR}/${client_DIRECTORY}
      -DROS_LIB_DIR=${${PROJECT_NAME}_ROS_LIB_DIR}
      -DEXECUTABLE_OUTPUT_PATH=${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}
      -DARDUINO_SDK_PATH=${TEENSYDUINO_SDK_PATH}
      -DTEENSYDUINO=${TEENSYDUINO}
      ${DTOOLCHAIN_FILE}
  )
  add_dependencies(${PROJECT_NAME}_${client_DIRECTORY} ${PROJECT_NAME}_ros_lib)
endfunction()
