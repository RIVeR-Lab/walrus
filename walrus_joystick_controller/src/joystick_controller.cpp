#include "walrus_joystick_controller/joystick_controller.h"
#include "walrus_joystick_controller/JoystickControllerState.h"
#include <walrus_pod_controller/PodCommand.h>
#include <walrus_drive_controller/TankDriveCommand.h>

namespace walrus_joystick_controller {

Pod::Pod(ros::NodeHandle nh, const std::string& topic) {
  pub_ = nh.advertise<walrus_pod_controller::PodCommand>(topic, 1);
}

void Pod::publishCommand(int mode, double set_point) {
  walrus_pod_controller::PodCommand command_msg;
  command_msg.set_point = set_point;
  command_msg.mode = mode;

  pub_.publish(command_msg);
}

void Pod::publishEffortOrHold(double effort) {
  if(effort == 0.0) {
    publishCommand(walrus_pod_controller::PodCommand::HOLD_POSITION, 0.0);
  }
  else {
    publishCommand(walrus_pod_controller::PodCommand::EFFORT, effort);
  }
}

void Pod::publishHold() {
  publishCommand(walrus_pod_controller::PodCommand::HOLD_POSITION, 0.0);
}



JoystickController::JoystickController(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : back_left_pod_(nh, "left_pods_joint_controller/back/command"),
    back_right_pod_(nh, "right_pods_joint_controller/back/command"),
    front_left_pod_(nh, "left_pods_joint_controller/front/command"),
    front_right_pod_(nh, "right_pods_joint_controller/front/command")
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

  pnh.param<int>("button_left_pods", button_left_pods_, 0);
  pnh.param<int>("button_right_pods", button_right_pods_, 1);

  pnh.param<int>("button_toggle_speed", button_toggle_speed_, 2);

  tank_drive_pub_ = nh.advertise<walrus_drive_controller::TankDriveCommand>("tank_drive", 1);
  
  boom_pan_effort = nh.advertise<std_msgs::Float64>("/boom/pan_controller/command", 1);
  boom_tilt_effort = nh.advertise<std_msgs::Float64>("/boom/tilt_controller/command", 1);
  boom_deploy_effort = nh.advertise<std_msgs::Float64>("/boom/deploy_controller/command", 1);

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
    double left_raw = joy_msg->axes[axis_tank_left_];
    double left_sign = left_raw < 0 ? -1 : 1;
    double right_raw = joy_msg->axes[axis_tank_right_];
    double right_sign = right_raw < 0 ? -1 : 1;
    tank_drive_msg.left_speed = left_sign * left_raw * left_raw * speed_scale;
    tank_drive_msg.right_speed = right_sign * right_raw * right_raw * speed_scale;
    tank_drive_pub_.publish(tank_drive_msg);

    bool front_up = joy_msg->buttons[button_front_pods_up_];
    bool back_up = joy_msg->buttons[button_back_pods_up_];
    bool front_down = joy_msg->buttons[button_front_pods_down_];
    bool back_down = joy_msg->buttons[button_back_pods_down_];

    bool left = joy_msg->buttons[button_left_pods_];
    bool right = joy_msg->buttons[button_right_pods_];

    double left_effort = 0.0;
    double right_effort = 0.0;
    if(left && !right) {
      left_effort = 1.0;
      right_effort = 0.0;
    }
    else if(!left && right) {
      left_effort = 0.0;
      right_effort = 1.0;
    }
    else {
      left_effort = 1.0;
      right_effort = 1.0;
    }

    double front_effort = 0.0;
    double back_effort = 0.0;
    if(front_up) {
      front_effort = -1.0;
    }
    else if(front_down) {
      front_effort = 1.0;
    }
    else {
      front_effort = 0.0;
    }

    if(back_up) {
      back_effort = 1.0;
    }
    else if(back_down) {
      back_effort = -1.0;
    }
    else {
      back_effort = 0.0;
    }

    back_left_pod_.publishEffortOrHold(back_effort * left_effort);
    back_right_pod_.publishEffortOrHold(back_effort * right_effort);
    front_left_pod_.publishEffortOrHold(front_effort * left_effort);
    front_right_pod_.publishEffortOrHold(front_effort * right_effort);
    std_msgs::Float64 pan, deploy;
    pan.data = joy_msg->axes[4];
    deploy.data = joy_msg->axes[5];
    boom_deploy_effort.publish(deploy);
    boom_pan_effort.publish(pan);
  }
}


void JoystickController::enableCallback(const std_msgs::Bool::ConstPtr& bool_msg)
{
  enabled_ = bool_msg->data;

  if(enabled_) {
    walrus_drive_controller::TankDriveCommand tank_drive_msg;
    tank_drive_msg.left_speed = 0.0;
    tank_drive_msg.right_speed = 0.0;
    tank_drive_pub_.publish(tank_drive_msg);

    back_left_pod_.publishHold();
    back_right_pod_.publishHold();
    front_left_pod_.publishHold();
    front_right_pod_.publishHold();
  }
  updateState(true);
}

}

