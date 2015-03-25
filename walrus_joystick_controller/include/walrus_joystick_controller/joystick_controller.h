#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Joy.h"

namespace walrus_joystick_controller {

class JoystickController {
 public:
  JoystickController(ros::NodeHandle& nh, ros::NodeHandle& pnh);

 private:
  void updateState(bool force_publish=false);
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);
  void enableCallback(const std_msgs::Bool::ConstPtr& bool_msg);

  ros::Publisher tank_drive_pub_;

  ros::Publisher back_left_pod_pub_;
  ros::Publisher back_right_pod_pub_;
  ros::Publisher front_left_pod_pub_;
  ros::Publisher front_right_pod_pub_;

  ros::Publisher state_pub_;
  ros::Timer state_pub_timer_;

  ros::Subscriber joy_sub_;
  ros::Subscriber enable_sub_;

  int axis_tank_left_;
  int axis_tank_right_;

  int button_front_pods_up_;
  int button_front_pods_down_;
  int button_back_pods_up_;
  int button_back_pods_down_;

  int button_toggle_speed_;
  bool previous_button_toggle_speed_state_;

  double high_speed_max_;
  double low_speed_max_;

  bool enabled_;
  bool high_speed_mode_;

  void publishFrontPodEffort(double effort);
  void publishBackPodEffort(double effort);
  void publishFrontPodHold();
  void publishBackPodHold();

  ros::Duration state_publish_delay_;
  ros::Time last_state_publish_;

  ros::Duration joystick_available_timeout_;
  ros::Time last_joy_message_;
};

}
