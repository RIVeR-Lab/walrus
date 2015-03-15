#include <ros/ros.h>
#include <ros/spinner.h>
#include "epos_hardware/epos_hardware.h"
#include <controller_manager/controller_manager.h>
#include <vector>
#include "walrus_base_hw/realtime_rate.h"
#include <boost/assign/list_of.hpp>
#include <controller_manager_msgs/SwitchController.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "walrus_base_epos_hw_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  double controller_rate;
  pnh.param<double>("controller_rate", controller_rate, 10);


  std::vector<std::string> epos_names;
  epos_names.push_back("left_drive_actuator");
  epos_names.push_back("right_drive_actuator");
  epos_hardware::EposHardware robot(nh, pnh, epos_names);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("Initializing Motors");
  if(!robot.init()) {
    ROS_FATAL("Failed to initialize motors");
    return 1;
  }
  ROS_INFO("Motors Initialized");

  std::vector<std::string> controller_names = boost::assign::list_of
    ("joint_state_controller")
    ("drive_controller");
  controller_manager::ControllerManager cm(&robot, pnh);
  BOOST_FOREACH(const std::string& name, controller_names) {
    if(!cm.loadController(name))
      ROS_ERROR_STREAM("Failed to load controller: " << name);
  }
  if(!cm.switchController(controller_names, std::vector<std::string>(), controller_manager_msgs::SwitchController::Request::BEST_EFFORT))
    ROS_ERROR("Failed to activate controllers");

  walrus_base_hw::RealtimeRate rate(controller_rate);
  while (ros::ok()) {
    ros::Duration dt;
    ros::Time now;
    rate.beginLoop(&now, &dt);

    robot.read();
    cm.update(now, dt);
    robot.write();

    robot.update_diagnostics();

    rate.sleep();
  }
  return 0;
}
