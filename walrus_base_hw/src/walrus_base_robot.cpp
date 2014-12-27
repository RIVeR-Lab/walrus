#include "walrus_base_hw/walrus_base_robot.h"

namespace walrus_base_hw {

WalrusBaseRobot::WalrusBaseRobot(ros::NodeHandle nh) : nh_(nh) {
}

bool WalrusBaseRobot::init() {
  try {
    transmission_loader_.reset(new TransmissionInterfaceLoader(this, &robot_transmissions_));
  }
  catch(const std::invalid_argument& ex){
    ROS_ERROR_STREAM("Failed to create transmission interface loader. " << ex.what());
    return false;
  }
  catch(const pluginlib::LibraryLoadException& ex){
    ROS_ERROR_STREAM("Failed to create transmission interface loader. " << ex.what());
    return false;
  }
  catch(...){
    ROS_ERROR_STREAM("Failed to create transmission interface loader. ");
    return false;
  }

  // Register ros_control interfaces
  registerInterface(&as_interface_);
  registerInterface(&ae_interface_);
  registerInterface(&av_interface_);


  // Load the robot description
  std::string urdf_string;
  nh_.getParam("robot_description", urdf_string);
  while (urdf_string.empty() && ros::ok()){
    ROS_INFO_STREAM_ONCE("Waiting for robot_description");
    nh_.getParam("robot_description", urdf_string);
    ros::Duration(0.2).sleep();
  }

  // Load transmissions from the robot description
  if (!transmission_loader_->load(urdf_string)) { return false; }

  return true;
}

// Write controller output to actuators
void WalrusBaseRobot::write(){
  robot_transmissions_.get<JointToActuatorEffortInterface>()->propagate();
  // Write actuator commands
}

// Read robot state
void WalrusBaseRobot::read(){
  // Read actuator commands
  robot_transmissions_.get<ActuatorToJointStateInterface>()->propagate();
}



}
