#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Joy.h"

namespace walrus_joystick_controller {

class JoystickController {
 public:
  JoystickController(ros::NodeHandle& nh, ros::NodeHandle& pnh);

 private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);
  void enabledCallback(const std_msgs::Bool::ConstPtr& bool_msg);

  ros::Subscriber joy_sub_;
  ros::Subscriber enabled_sub_;

  ros::Publisher tank_drive_pub_;

  ros::Publisher back_left_pod_pub_;
  ros::Publisher back_right_pod_pub_;
  ros::Publisher front_left_pod_pub_;
  ros::Publisher front_right_pod_pub_;

  int axis_tank_left_;
  int axis_tank_right_;

  int button_front_pods_up_;
  int button_front_pods_down_;
  int button_back_pods_up_;
  int button_back_pods_down_;

  double scale_linear_;

  bool enabled_;

  void publishFrontPodEffort(double effort);
  void publishBackPodEffort(double effort);
  void publishFrontPodHold();
  void publishBackPodHold();
};

}
