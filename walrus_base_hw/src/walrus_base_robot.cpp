#include "walrus_base_hw/walrus_base_robot.h"
#include <boost/shared_ptr.hpp>

namespace walrus_base_hw {

void WalrusBaseRobot::createFakeActuator(const std::string& name) {
  boost::shared_ptr<FakeActuatorData> data(new FakeActuatorData(name));
  hardware_interface::ActuatorStateHandle state_handle(name, &data->position, &data->velocity, &data->effort);
  as_interface_.registerHandle(state_handle);

  hardware_interface::ActuatorHandle position_handle(state_handle, &data->cmd);
  ap_interface_.registerHandle(position_handle);
  hardware_interface::ActuatorHandle velocity_handle(state_handle, &data->cmd);
  av_interface_.registerHandle(velocity_handle);

  fake_actuator_data.push_back(data);
}


WalrusBaseRobot::WalrusBaseRobot(ros::NodeHandle nh, ros::NodeHandle pnh)
  : nh_(nh), pnh_(pnh),
    epos_manager_(as_interface_, av_interface_, ap_interface_, nh, pnh) {

  XmlRpc::XmlRpcValue epos_motors_xml;
  if(pnh_.getParam("epos_motors", epos_motors_xml)) {
    epos_manager_.load(epos_motors_xml);
  }
  else {
    ROS_FATAL("No EPOS motor definitions found");
  }
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

  if(!epos_manager_.init()) {
    ROS_ERROR("Failed to initialize EPOS");
    return false;
  }

  // Create fake actuators so that transmission loading doesn't fail
  createFakeActuator("walrus/back_left_pod_joint_actuator");
  createFakeActuator("walrus/front_left_pod_joint_actuator");
  createFakeActuator("walrus/back_right_pod_joint_actuator");
  createFakeActuator("walrus/front_right_pod_joint_actuator");

  createFakeActuator("walrus/boom/deploy_joint_actuator");
  createFakeActuator("walrus/boom/pan_joint_actuator");
  createFakeActuator("walrus/boom/tilt_joint_actuator");


  // Register ros_control interfaces
  registerInterface(&as_interface_);
  registerInterface(&av_interface_);
  registerInterface(&ap_interface_);


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
  robot_transmissions_.get<JointToActuatorPositionInterface>()->propagate();
  robot_transmissions_.get<JointToActuatorVelocityInterface>()->propagate();

  // Write actuator commands
  epos_manager_.write();

  // Print fake actuator commands
  static ros::Time last = ros::Time::now();
  if(ros::Time::now() - last >= ros::Duration(5.0)) {
    last = ros::Time::now();
    ROS_INFO_STREAM("------------------------------------");
    BOOST_FOREACH(const boost::shared_ptr<FakeActuatorData>& data, fake_actuator_data) {
      ROS_INFO_STREAM("Fake Actuator (" << data->name << ") Cmd: " << data->cmd);
    }
    ROS_INFO_STREAM("------------------------------------");
  }

}

// Read robot state
void WalrusBaseRobot::read(){
  // Read actuator commands
  epos_manager_.read();

  robot_transmissions_.get<ActuatorToJointStateInterface>()->propagate();
}

void WalrusBaseRobot::update_diagnostics(){
  epos_manager_.update_diagnostics();
}


}
