#include <ros/ros.h>
#include <ros/spinner.h>
#include "walrus_base_hw/walrus_base_robot.h"
#include <controller_manager/controller_manager.h>
#include <boost/asio.hpp>
#include <rosserial_server/serial_session.h>
#if HAVE_BOOST_CHRONO
#include <boost/chrono/system_clocks.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#endif

#define NSEC_PER_SEC (1e9d)

// Node running robot hardware interfaces and controllers
int main( int argc, char** argv ){
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
  controller_manager::ControllerManager cm(&robot, nh);

  // Startup ROS spinner in background
  ros::AsyncSpinner spinner(1);
  spinner.start();

  if(!robot.init()) {
    ROS_FATAL("Failed to initailize robot");
    return 1;
  }


#if HAVE_BOOST_CHRONO
  ROS_INFO_ONCE("Using Boost chrono for timing");
  boost::chrono::nanoseconds controller_period((long)(NSEC_PER_SEC/controller_rate));
  boost::chrono::steady_clock::time_point next_time = boost::chrono::steady_clock::now(); // store the next time the controller loop should run
  bool first_period = true;
  boost::chrono::steady_clock::time_point previous_time;
#else
  ROS_INFO_ONCE("Falling back to rostime for timing");
  ros::Rate controller_ros_rate(controller_rate);
  ros::Time last = ros::Time::now();
#endif
  while (ros::ok()) {
    robot.read();
#if HAVE_BOOST_CHRONO
    boost::chrono::steady_clock::time_point now = boost::chrono::steady_clock::now();
    if(first_period) {
      first_period = false;
      previous_time = now;
    }
    boost::chrono::nanoseconds time_since_last = now - previous_time;
    ros::Duration dt(time_since_last.count() / NSEC_PER_SEC);
#else
    ros::Time now = ros::Time::now();
    ros::Duration dt = now - last;
#endif
    cm.update(ros::Time::now(), dt);
    robot.write();

    robot.update_diagnostics();


#if HAVE_BOOST_CHRONO
    boost::chrono::nanoseconds update_time = boost::chrono::steady_clock::now()-now;
    ROS_DEBUG_THROTTLE(1.0, "Controller update took %dns", update_time.count());

    previous_time = now;
    next_time = next_time + controller_period;

    boost::chrono::nanoseconds sleep_time = next_time - boost::chrono::steady_clock::now();
    if(sleep_time.count() > 0)
      boost::this_thread::sleep(boost::posix_time::microseconds(boost::chrono::duration_cast<boost::chrono::microseconds>(sleep_time).count()));
    else
      ROS_ERROR_THROTTLE(0.5, "Controller update took longer than loop rate!!!");
#else
    last = now;
    controller_ros_rate.sleep();
#endif
  }

}
