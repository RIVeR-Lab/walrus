#include "walrus_joystick_controller/joystick_controller.h"
#include "walrus_joystick_controller/JoystickControllerState.h"
#include <walrus_pod_controller/PodCommand.h>
#include <walrus_drive_controller/TankDriveCommand.h>

namespace walrus_joystick_controller {


JoystickController::JoystickController(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  state_publish_delay_ = ros::Duration(1.0);
  last_state_publish_ = ros::Time(0);

  joystick_available_timeout_ = ros::Duration(2.0);
  last_joy_message_ = ros::Time(0);

  previous_button_toggle_speed_state_ = false;

  high_speed_mode_ = false;

  pnh.param<bool>("enable_by_default", enabled_, false);

  pnh.param<int>("axis_tank_left", axis_tank_left_, 1);
  pnh.param<int>("axis_tank_right", axis_tank_right_, 3);

  pnh.param<double>("high_speed_max", high_speed_max_, 2.0);
  pnh.param<double>("low_speed_max", low_speed_max_, 0.5);

  pnh.param<int>("button_front_pods_up", button_front_pods_up_, 0);
  pnh.param<int>("button_front_pods_down", button_front_pods_down_, 1);
  pnh.param<int>("button_back_pods_up", button_back_pods_up_, 0);
  pnh.param<int>("button_back_pods_down", button_back_pods_down_, 1);

  pnh.param<int>("button_toggle_speed", button_toggle_speed_, 2);

  tank_drive_pub_ = nh.advertise<walrus_drive_controller::TankDriveCommand>("tank_drive", 1);

  back_left_pod_pub_ = nh.advertise<walrus_pod_controller::PodCommand>("left_pods_joint_controller/back/command", 1);
  back_right_pod_pub_ = nh.advertise<walrus_pod_controller::PodCommand>("right_pods_joint_controller/back/command", 1);
  front_left_pod_pub_ = nh.advertise<walrus_pod_controller::PodCommand>("left_pods_joint_controller/front/command", 1);
  front_right_pod_pub_ = nh.advertise<walrus_pod_controller::PodCommand>("right_pods_joint_controller/front/command", 1);

  state_pub_ = pnh.advertise<walrus_joystick_controller::JoystickControllerState>("state", 1, true);
  state_pub_timer_ = nh.createTimer(ros::Duration(1.0), boost::bind(&JoystickController::updateState, this, false));

  joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 1, &JoystickController::joyCallback, this);
  enable_sub_ = nh.subscribe<std_msgs::Bool>("enable", 1, &JoystickController::enableCallback, this);
}

void JoystickController::updateState(bool force_publish) {
  bool available = last_joy_message_ + joystick_available_timeout_ > ros::Time::now();
  if(last_state_publish_ + state_publish_delay_ < ros::Time::now() || !available || force_publish) {
    walrus_joystick_controller::JoystickControllerState state;
    state.enabled = enabled_;
    state.high_speed_mode = high_speed_mode_;
    state.available = available;
    state_pub_.publish(state);
    last_state_publish_ = ros::Time::now();
  }
}

void JoystickController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  last_joy_message_ = ros::Time::now();
  updateState();
  if(enabled_)
  {
    bool button_toggle_speed_state = joy_msg->buttons[button_toggle_speed_];
    // if button state changed and was pressed in previous message (button was just released)
    if(previous_button_toggle_speed_state_ != button_toggle_speed_state
       && previous_button_toggle_speed_state_) {
      high_speed_mode_ = !high_speed_mode_;
      updateState(true);
    }
    previous_button_toggle_speed_state_ = button_toggle_speed_state;

    double speed_scale = high_speed_mode_ ? high_speed_max_ : low_speed_max_;
    walrus_drive_controller::TankDriveCommand tank_drive_msg;
    tank_drive_msg.left_speed = joy_msg->axes[axis_tank_left_]* speed_scale;
    tank_drive_msg.right_speed = joy_msg->axes[axis_tank_right_]* speed_scale;
    tank_drive_pub_.publish(tank_drive_msg);


    if(joy_msg->buttons[button_front_pods_up_]) {
      publishFrontPodEffort(-1.0);
    }
    else if(joy_msg->buttons[button_front_pods_down_]) {
      publishFrontPodEffort(1.0);
    }
    else {
      publishFrontPodHold();
    }

    if(joy_msg->buttons[button_back_pods_up_]) {
      publishBackPodEffort(1.0);
    }
    else if(joy_msg->buttons[button_back_pods_down_]) {
      publishBackPodEffort(-1.0);
    }
    else {
      publishBackPodHold();
    }
  }
}

void JoystickController::publishFrontPodEffort(double effort) {
  walrus_pod_controller::PodCommand effort_msg;
  effort_msg.set_point = effort;
  effort_msg.mode = walrus_pod_controller::PodCommand::EFFORT;

  front_left_pod_pub_.publish(effort_msg);
  front_right_pod_pub_.publish(effort_msg);
}
void JoystickController::publishBackPodEffort(double effort) {
  walrus_pod_controller::PodCommand effort_msg;
  effort_msg.set_point = effort;
  effort_msg.mode = walrus_pod_controller::PodCommand::EFFORT;

  back_left_pod_pub_.publish(effort_msg);
  back_right_pod_pub_.publish(effort_msg);
}
void JoystickController::publishBackPodHold() {
  walrus_pod_controller::PodCommand effort_msg;
  effort_msg.mode = walrus_pod_controller::PodCommand::HOLD_POSITION;

  back_left_pod_pub_.publish(effort_msg);
  back_right_pod_pub_.publish(effort_msg);
}
void JoystickController::publishFrontPodHold() {
  walrus_pod_controller::PodCommand effort_msg;
  effort_msg.mode = walrus_pod_controller::PodCommand::HOLD_POSITION;

  front_left_pod_pub_.publish(effort_msg);
  front_right_pod_pub_.publish(effort_msg);
}


void JoystickController::enableCallback(const std_msgs::Bool::ConstPtr& bool_msg)
{
  enabled_ = bool_msg->data;

  if(enabled_) {
    walrus_drive_controller::TankDriveCommand tank_drive_msg;
    tank_drive_msg.left_speed = 0.0;
    tank_drive_msg.right_speed = 0.0;
    tank_drive_pub_.publish(tank_drive_msg);

    walrus_pod_controller::PodCommand hold_msg;
    hold_msg.mode = walrus_pod_controller::PodCommand::HOLD_POSITION;
    back_left_pod_pub_.publish(hold_msg);
    back_right_pod_pub_.publish(hold_msg);
    front_left_pod_pub_.publish(hold_msg);
    front_right_pod_pub_.publish(hold_msg);
  }
  updateState(true);
}

}

