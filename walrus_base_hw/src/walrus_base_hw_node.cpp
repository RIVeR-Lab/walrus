#include <ros/ros.h>
#include <ros/spinner.h>
#include "walrus_base_hw/walrus_base_robot.h"
#include <controller_manager/controller_manager.h>

int main( int argc, char** argv ){
  ros::init(argc, argv, "walrus_base_hw");
  ros::NodeHandle nh;

  walrus_base_hw::WalrusBaseRobot robot;
  controller_manager::ControllerManager cm(&robot, nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  robot.init();

  //won't run this is simulation so regular time is ok
  ros::Rate controller_rate(50);
  ros::Time last = ros::Time::now();
  while (ros::ok()) {
    robot.read();
    ros::Time now = ros::Time::now();
    cm.update(now, now-last);
    robot.write();
    last = now;
    controller_rate.sleep();
  }

}
