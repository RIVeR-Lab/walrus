#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

namespace walrus_joystick_controller {

class JoystickController {
 public:
  JoystickController(ros::NodeHandle& nh, ros::NodeHandle& pnh);

 private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);
  void enabledCallback(const std_msgs::Bool::ConstPtr& bool_msg);

  ros::Subscriber joy_sub_;
  ros::Subscriber enabled_sub_;
  ros::Publisher cmd_vel_pub_;

  int axis_linear_;
  int axis_angular_;

  double scale_linear_;
  double scale_angular_;

  bool enabled_;
};

}
