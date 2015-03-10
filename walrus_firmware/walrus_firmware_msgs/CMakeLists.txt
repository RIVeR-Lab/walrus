cmake_minimum_required(VERSION 2.8.3)
project(walrus_firmware_msgs)

find_package(catkin REQUIRED COMPONENTS
	message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
<<<<<<< HEAD
  BoomBoardHighSpeedControl.msg
  BoomBoardHighSpeedFeedback.msg
  BoomBoardLowSpeedData.msg
  BoomBoardControl.msg
  DiagnosticRXMsg.msg
  DiagnosticTXMsg.msg
  MainBoardPodMotorFeedback.msg
  MainBoardSensorData.msg
=======
  BoomBoardRXMsg.msg
  BoomBoardTXMsg.msg
  DiagnosticScreen.msg
  DiagnosticButton.msg
  MainBoardHighSpeedControl.msg
  MainBoardHighSpeedFeedback.msg
  MainBoardLowSpeedData.msg
>>>>>>> Renamed Diaggnostic board messages to be more descriptive, added non standard and custom characters as constants to DiagnosticScreen message
  MainBoardControl.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS message_runtime
)
