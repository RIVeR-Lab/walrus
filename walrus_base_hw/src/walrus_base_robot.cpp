#include "walrus_base_hw/walrus_base_robot.h"
#include "walrus_base_hw/util.h"
#include <boost/shared_ptr.hpp>
#include <boost/assign/list_of.hpp>

namespace walrus_base_hw {

WalrusBaseRobot::WalrusBaseRobot(ros::NodeHandle nh, ros::NodeHandle pnh)
  : nh_(nh), pnh_(pnh),
    mainboard_(as_interface_, ae_interface_, nh, pnh),
    boomboard_(as_interface_, ae_interface_, nh, pnh) {
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
  registerInterface(&ae_interface_);


  // Load the robot description
  std::string urdf_string;
  nh_.getParam("robot_description", urdf_string);
  while (urdf_string.empty() && ros::ok()){
    ROS_INFO_STREAM_ONCE("Waiting for robot_description");
    nh_.getParam("robot_description", urdf_string);
    ros::Duration(0.2).sleep();
  }

  // Load transmissions
  std::vector<std::string> actuator_names = boost::assign::list_of
    ("walrus/front_left_pod_joint_actuator")
    ("walrus/front_right_pod_joint_actuator")
    ("walrus/back_left_pod_joint_actuator")
    ("walrus/back_right_pod_joint_actuator")
    ("walrus/boom/deploy_joint_actuator")
    ("walrus/boom/pan_joint_actuator")
    ("walrus/boom/tilt_joint_actuator");
  if (!loadTransmissions(urdf_string, actuator_names, transmission_loader_.get())) { return false; }

  return true;
}

// Write controller output to actuators
void WalrusBaseRobot::write(ros::Duration dt){
  robot_transmissions_.get<JointToActuatorEffortInterface>()->propagate();

  // Write actuator commands
  mainboard_.write(dt);
  boomboard_.write(dt);

}

// Read robot state
void WalrusBaseRobot::read(ros::Duration dt){
  // Read actuator commands
  mainboard_.read(dt);
  boomboard_.read(dt);

  robot_transmissions_.get<ActuatorToJointStateInterface>()->propagate();
}

void WalrusBaseRobot::update_diagnostics(){
  mainboard_.update_diagnostics();
  boomboard_.update_diagnostics();
}


}
