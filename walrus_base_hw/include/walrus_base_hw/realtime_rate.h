#ifndef WALRUS_BASE_HW_REALTIME_RATE_H_
#define WALRUS_BASE_HW_REALTIME_RATE_H_

#include <ros/ros.h>
#if HAVE_BOOST_CHRONO
#include <boost/chrono/system_clocks.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#endif

#define NSEC_PER_SEC (1e9d)

namespace walrus_base_hw {

class RealtimeRate {
public:
  RealtimeRate(double rate)
    : first_period_(true),
#if HAVE_BOOST_CHRONO
      controller_period_((long)(NSEC_PER_SEC/rate))
#else
      controller_ros_rate_(rate)
#endif
  {
#if HAVE_BOOST_CHRONO
    ROS_INFO_ONCE("Using Boost chrono for timing");
#else
    ROS_INFO_ONCE("Falling back to rostime for timing");
#endif
  }

#if HAVE_BOOST_CHRONO
  typedef boost::chrono::steady_clock::time_point TimeInstant;
  typedef boost::chrono::nanoseconds TimeDuration;
#define TO_ROS_DURATION(d) (ros::Duration((d).count() / NSEC_PER_SEC))
#define NOW() (boost::chrono::steady_clock::now())
#else
  typedef ros::Time TimeInstant;
  typedef ros::Duration TimeDuration;
#define TO_ROS_DURATION(d) (d)
#define NOW() (ros::Time::now())
#endif

  void beginLoop(ros::Time* time, ros::Duration* duration) {
    TimeInstant last_loop_begin_ = loop_begin_;
    loop_begin_ = NOW();

    *time = ros::Time::now();

    if(first_period_) {
      *duration = ros::Duration(0);
      first_period_ = false;
      return;
    }

    *duration = TO_ROS_DURATION(loop_begin_ - last_loop_begin_);
  }

  void sleep() {
    TimeInstant now = NOW();
    TimeDuration update_time = loop_begin_ - now;
    ROS_DEBUG_THROTTLE(1.0, "Controller update took %fs", TO_ROS_DURATION(update_time).toSec());

#if HAVE_BOOST_CHRONO
    TimeInstant next_loop_begin = loop_begin_ + controller_period_;
    TimeDuration sleep_time = next_loop_begin - now;
    if(sleep_time.count() > 0)
      boost::this_thread::sleep(boost::posix_time::microseconds(boost::chrono::duration_cast<boost::chrono::microseconds>(sleep_time).count()));
    else
      ROS_ERROR_THROTTLE(0.5, "Controller update took longer than loop rate!!!");
#else
    controller_ros_rate_.sleep();
#endif
  }

private:
  bool first_period_;
  TimeInstant loop_begin_;

#if HAVE_BOOST_CHRONO
  boost::chrono::nanoseconds controller_period_;
#else
  ros::Rate controller_ros_rate_;
#endif
};

}

#endif
