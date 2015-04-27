#include <ros/ros.h>
#include <ros/spinner.h>
#include <walrus_pod_hw/walrus_pod_hw.h>
#include <controller_manager/controller_manager.h>
#include <vector>
#include "walrus_base_hw/realtime_rate.h"
#include "walrus_base_hw/util.h"
#include <boost/assign/list_of.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "pod_hw_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  double controller_rate;
  pnh.param<double>("controller_rate", controller_rate, 50);

  walrus_pod_hw::WalrusPodHW robot(nh, pnh, "/dev/walrus_front_pod_controller", "/dev/walrus_back_pod_controller");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("Initializing Pods");
  if(!robot.init()) {
    ROS_FATAL("Failed to initialize Pods");
    return 1;
  }
  ROS_INFO("Pods Initialized");

  controller_manager::ControllerManager cm(&robot, pnh);
  ros::Timer controller_load_timer = walrus_base_hw::createControllerLoadTimer(pnh, &cm);

  ROS_INFO("Running loop");
  walrus_base_hw::RealtimeRate rate(controller_rate);
  while (ros::ok()) {
    ros::Duration dt;
    ros::Time now;
    rate.beginLoop(&now, &dt);

    robot.read(dt);
    cm.update(now, dt);
    robot.write(dt);

    robot.update_diagnostics();

    rate.sleep();
  }
  return 0;
}
