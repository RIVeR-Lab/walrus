#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

namespace walrus_joystick_controller {

class Pod {
public:
  Pod(ros::NodeHandle nh, const std::string& topic);
  void publishCommand(int mode, double set_point);
  void publishEffortOrHold(double effort);
  void publishHold();
private:
  ros::Publisher pub_;
};

template <class D, class M>
class DataPublisher {
public:
  void advertise(ros::NodeHandle nh, const std::string& topic, int queue_size) {
    pub_ = nh.advertise<M>(topic, queue_size);
  }
  void publish(const D& data) {
    M msg;
    msg.data = data;
    pub_.publish(msg);
  }
private:
  ros::Publisher pub_;
};

typedef DataPublisher<double, std_msgs::Float64> Float64Publisher;
typedef DataPublisher<bool, std_msgs::Bool> BoolPublisher;



class JoystickController {
 public:
  JoystickController(ros::NodeHandle& nh, ros::NodeHandle& pnh);

 private:
  void updateState(bool force_publish=false);
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);
  void enableCallback(const std_msgs::Bool::ConstPtr& bool_msg);

  ros::Publisher tank_drive_pub_;

  Pod back_left_pod_;
  Pod back_right_pod_;
  Pod front_left_pod_;
  Pod front_right_pod_;

  ros::Publisher boom_tilt_effort, boom_pan_effort, boom_deploy_effort;

  Float64Publisher arm_tilt_pub, arm_pan_pub, arm_shoulder_pub;
  BoolPublisher spray_pub;

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

  int button_left_pods_;
  int button_right_pods_;

  int axis_boom_pan_;
  int axis_boom_tilt_;
  int axis_boom_deploy_;
  int button_boom_deploy_enable_;

  int button_arm_enable_;
  int button_arm_spray_;
  int axis_arm_pan_;
  int axis_arm_shoulder_;
  int axis_arm_tilt_;

  int button_toggle_speed_;
  bool previous_button_toggle_speed_state_;

  double high_speed_max_;
  double low_speed_max_;

  bool enabled_;
  bool high_speed_mode_;

  ros::Duration state_publish_delay_;
  ros::Time last_state_publish_;

  ros::Duration joystick_available_timeout_;
  ros::Time last_joy_message_;
};

}
