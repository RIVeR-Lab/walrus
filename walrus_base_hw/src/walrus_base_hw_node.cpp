#include <ros/ros.h>
#include <ros/spinner.h>
#include "walrus_base_hw/walrus_base_robot.h"
#include "walrus_base_hw/realtime_rate.h"
#include <controller_manager/controller_manager.h>
#include <boost/asio.hpp>
#include <rosserial_server/serial_session.h>
#include <boost/assign/list_of.hpp>
#include <controller_manager_msgs/SwitchController.h>

void switch_controllers(controller_manager::ControllerManager* cm) {
  std::vector<std::string> controller_names = boost::assign::list_of
    ("joint_state_controller")
    ("left_pods_joint_controller")
    ("right_pods_joint_controller")
    ("boom_deploy_controller")
    ("boom_pan_controller")
    ("boom_tilt_controller");
  BOOST_FOREACH(const std::string& name, controller_names) {
    ROS_INFO_STREAM("Loading " << name);
    if(!cm->loadController(name))
      ROS_ERROR_STREAM("Failed to load controller: " << name);
  }

  if(!cm->switchController(controller_names, std::vector<std::string>(), controller_manager_msgs::SwitchController::Request::BEST_EFFORT))
    ROS_ERROR("Failed to activate controllers");
}

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
  // need to activate controllers async so that the control loop will run concurrently
  ros::Timer startControllerTimer = nh.createTimer(ros::Duration(0), boost::bind(switch_controllers, &cm), true);

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
