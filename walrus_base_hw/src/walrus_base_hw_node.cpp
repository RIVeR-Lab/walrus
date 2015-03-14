#include <ros/ros.h>
#include <ros/spinner.h>
#include "walrus_base_hw/walrus_base_robot.h"
#include "walrus_base_hw/realtime_rate.h"
#include <controller_manager/controller_manager.h>
#include <boost/asio.hpp>
#include <rosserial_server/serial_session.h>

int main( int argc, char** argv ) {
  ros::init(argc, argv, "walrus_base_hw");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  double controller_rate;
  pnh.param<double>("controller_rate", controller_rate, 50);

  std::string mainboard_port;
  pnh.param<std::string>("mainboard_port", mainboard_port, "/dev/walrus_main_board");
  int mainboard_baud;
  pnh.param("mainboard_baud", mainboard_baud, 57600);

  std::string boomboard_port;
  pnh.param<std::string>("boomboard_port", boomboard_port, "/dev/walrus_boom_board");
  int boomboard_baud;
  pnh.param("boomboard_baud", boomboard_baud, 57600);

  boost::asio::io_service io_service;
  // Destructor is private...
  new rosserial_server::SerialSession(io_service, mainboard_port, mainboard_baud);
  new rosserial_server::SerialSession(io_service, boomboard_port, boomboard_baud);

  boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));

  walrus_base_hw::WalrusBaseRobot robot(nh, pnh);

  // Startup ROS spinner in background
  ros::AsyncSpinner spinner(1);
  spinner.start();

  if(!robot.init()) {
    ROS_FATAL("Failed to initailize robot");
    return 1;
  }

  controller_manager::ControllerManager cm(&robot, pnh);

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
