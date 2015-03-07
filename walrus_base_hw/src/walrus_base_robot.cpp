#include "walrus_base_hw/walrus_base_robot.h"
#include <boost/shared_ptr.hpp>

namespace walrus_base_hw {

WalrusBaseRobot::WalrusBaseRobot(ros::NodeHandle nh, ros::NodeHandle pnh)
  : nh_(nh), pnh_(pnh),
    mainboard_(as_interface_, ae_interface_, nh, pnh),
    boomboard_(as_interface_, ae_interface_, nh, pnh){
  std::vector<std::string> epos_names;
  epos_names.push_back("left_drive_actuator");
  epos_names.push_back("right_drive_actuator");
  epos_manager_.reset(new epos_hardware::EposManager(as_interface_, av_interface_, ap_interface_, nh, pnh, epos_names));
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

  if(!epos_manager_->init()) {
    ROS_ERROR("Failed to initialize EPOS");
    //return false;
  }
  
  if (!mainboard_.init()) {
    ROS_ERROR("Failed to initialize Main Board");
    return false;
  }
  
  if (!boomboard_.init()) {
    ROS_ERROR("Failed to initialize Boom Board");
    return false;
  }

  // Register ros_control interfaces
  registerInterface(&as_interface_);
  registerInterface(&av_interface_);
  registerInterface(&ap_interface_);
  registerInterface(&ae_interface_);


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
  robot_transmissions_.get<JointToActuatorVelocityInterface>()->propagate();

  // Write actuator commands
  //epos_manager_->write();
  mainboard_.write();
  boomboard_.write();

}

// Read robot state
void WalrusBaseRobot::read(){
  // Read actuator commands
  //epos_manager_->read();
  mainboard_.read();
  boomboard_.read();

  robot_transmissions_.get<ActuatorToJointStateInterface>()->propagate();
}

void WalrusBaseRobot::update_diagnostics(){
  //epos_manager_->update_diagnostics();
  mainboard_.update_diagnostics();
  boomboard_.update_diagnostics();
}


}
