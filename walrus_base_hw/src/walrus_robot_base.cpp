#include "walrus_base_hw/walrus_robot_base.h"
#include "walrus_base_hw/util.h"
#include <boost/shared_ptr.hpp>
#include <boost/assign/list_of.hpp>

namespace walrus_base_hw {

WalrusRobotBase::WalrusRobotBase(ros::NodeHandle nh, ros::NodeHandle pnh)
  : nh_(nh), pnh_(pnh) {
}

bool WalrusRobotBase::loadTransmissions(const std::vector<std::string>& actuator_names) {
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

  // Load the robot description
  std::string urdf_string;
  nh_.getParam("robot_description", urdf_string);
  while (urdf_string.empty() && ros::ok()){
    ROS_INFO_STREAM_ONCE("Waiting for robot_description");
    nh_.getParam("robot_description", urdf_string);
    ros::Duration(0.2).sleep();
  }

  // Load transmissions
  if (!loadTransmissionsFromUrdf(urdf_string, actuator_names, transmission_loader_.get())) { return false; }

  return true;
}

}
