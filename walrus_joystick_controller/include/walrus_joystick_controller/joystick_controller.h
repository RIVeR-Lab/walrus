#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Joy.h"
#include <walrus_stair_detector/Stair.h>

namespace walrus_joystick_controller {

class ToggleButton {
public:
  ToggleButton(bool initial_state)
    : state_(initial_state),
      previous_button_state_(false) {}
  bool update(bool toggle_button_state) {
    if(previous_button_state_ != toggle_button_state
       && previous_button_state_) {
      state_ = !state_;
      previous_button_state_ = toggle_button_state;
      return true;
    }
    else {
      previous_button_state_ = toggle_button_state;
      return false;
    }
  }
  bool state() { return state_; }
private:
  bool state_;
  bool previous_button_state_;
};

template <typename M> class OldMessageBuffer {
public:
  OldMessageBuffer(const ros::Duration& timeout)
    :timeout_(timeout), last_msg_stamp_(0) {}
  void feed(typename M::ConstPtr msg) {
    last_msg_ = msg;
    last_msg_stamp_ = ros::Time::now();
  }
  bool available() {
    return !(last_msg_stamp_.isZero() || last_msg_stamp_ + timeout_ > ros::Time::now());
  }
  typename M::ConstPtr get() {
    if(!available())
      return typename M::ConstPtr();
    return last_msg_;
  }
private:
  typename M::ConstPtr last_msg_;
  ros::Time last_msg_stamp_;
  ros::Duration timeout_;
};

class Pod {
public:
  Pod(ros::NodeHandle nh, const std::string& topic);
  void publishCommand(int mode, double set_point);
  void publishEffortOrHold(double effort);
  void publishHold();
private:
  ros::Publisher pub_;
};

class JoystickController {
 public:
  JoystickController(ros::NodeHandle& nh, ros::NodeHandle& pnh);

 private:
  void updateState(bool force_publish=false);
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);
  void enableCallback(const std_msgs::Bool::ConstPtr& bool_msg);
  void stairCallback(const walrus_stair_detector::Stair::ConstPtr& stair_msg);

  ros::Publisher tank_drive_pub_;
  ros::Publisher twist_pub_;

  Pod back_left_pod_;
  Pod back_right_pod_;
  Pod front_left_pod_;
  Pod front_right_pod_;

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

  int button_toggle_speed_;
  int button_toggle_stair_;

  double high_speed_max_;
  double low_speed_max_;

  bool enabled_;
  ToggleButton high_speed_mode_;
  ToggleButton stair_mode_;

  ros::Duration state_publish_delay_;
  ros::Time last_state_publish_;

  OldMessageBuffer<sensor_msgs::Joy> joy_available_buffer_;
  OldMessageBuffer<walrus_stair_detector::Stair> stair_detection_buffer_;
};

}
