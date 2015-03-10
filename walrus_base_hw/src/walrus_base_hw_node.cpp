#include <ros/ros.h>
#include <ros/spinner.h>
#include "walrus_base_hw/walrus_base_robot.h"
#include <controller_manager/controller_manager.h>
#include <boost/chrono/system_clocks.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#define NSEC_PER_SEC (1e9d)

// Node running robot hardware interfaces and controllers
int main( int argc, char** argv ){
  ros::init(argc, argv, "walrus_base_hw");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  double controller_rate;
  pnh.param<double>("controller_rate", controller_rate, 50);

  walrus_base_hw::WalrusBaseRobot robot(nh, pnh);
  controller_manager::ControllerManager cm(&robot, nh);

  // Startup ROS spinner in background
  ros::AsyncSpinner spinner(1);
  spinner.start();

  if(!robot.init()) {
    ROS_FATAL("Failed to initailize robot");
    return 1;
  }


  //won't run this is simulation so regular time is ok
  boost::chrono::nanoseconds controller_period((long)(NSEC_PER_SEC/controller_rate));
  boost::chrono::steady_clock::time_point next_time = boost::chrono::steady_clock::now(); // store the next time the controller loop should run

  bool first_period = true;
  boost::chrono::steady_clock::time_point previous_time;
  while (ros::ok()) {
    boost::chrono::steady_clock::time_point now = boost::chrono::steady_clock::now();
    if(first_period) {
      first_period = false;
      previous_time = now;
    }
    boost::chrono::nanoseconds time_since_last = now - previous_time;
    robot.read();
    cm.update(ros::Time::now(), ros::Duration(time_since_last.count() / NSEC_PER_SEC));
    robot.write();

    previous_time = now;
    next_time = next_time + controller_period;

    robot.update_diagnostics();
    boost::chrono::nanoseconds update_time = boost::chrono::steady_clock::now()-now;
    ROS_DEBUG_THROTTLE(1.0, "Controller update took %dns", update_time.count());

    boost::chrono::nanoseconds sleep_time = next_time - boost::chrono::steady_clock::now();
    if(sleep_time.count() > 0)
      boost::this_thread::sleep(boost::posix_time::microseconds(boost::chrono::duration_cast<boost::chrono::microseconds>(sleep_time).count()));
    else
      ROS_ERROR_THROTTLE(0.5, "Controller update took longer than loop rate!!!");
  }

}
