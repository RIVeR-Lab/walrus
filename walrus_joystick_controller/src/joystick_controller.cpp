#include "walrus_joystick_controller/joystick_controller.h"
#include <walrus_pod_controller/PodCommand.h>
#include <walrus_drive_controller/TankDriveCommand.h>

namespace walrus_joystick_controller {


JoystickController::JoystickController(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  pnh.param<bool>("enable_by_default", enabled_, false);

  pnh.param<int>("axis_tank_left", axis_tank_left_, 1);
  pnh.param<int>("axis_tank_right", axis_tank_right_, 3);
  pnh.param<double>("scale_linear", scale_linear_, 0.5);

  pnh.param<int>("button_front_pods_up", button_front_pods_up_, 0);
  pnh.param<int>("button_front_pods_down", button_front_pods_down_, 1);
  pnh.param<int>("button_back_pods_up", button_back_pods_up_, 0);
  pnh.param<int>("button_back_pods_down", button_back_pods_down_, 1);

  joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 1, &JoystickController::joyCallback, this);
  enabled_sub_ = nh.subscribe<std_msgs::Bool>("enabled", 1, &JoystickController::enabledCallback, this);

  tank_drive_pub_ = nh.advertise<walrus_drive_controller::TankDriveCommand>("tank_drive", 1);

  back_left_pod_pub_ = nh.advertise<walrus_pod_controller::PodCommand>("left_pods_joint_controller/back/command", 1);
  back_right_pod_pub_ = nh.advertise<walrus_pod_controller::PodCommand>("right_pods_joint_controller/back/command", 1);
  front_left_pod_pub_ = nh.advertise<walrus_pod_controller::PodCommand>("left_pods_joint_controller/front/command", 1);
  front_right_pod_pub_ = nh.advertise<walrus_pod_controller::PodCommand>("right_pods_joint_controller/front/command", 1);
}

void JoystickController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  if(enabled_)
  {
    walrus_drive_controller::TankDriveCommand tank_drive_msg;
    tank_drive_msg.left_speed = joy_msg->axes[axis_tank_left_]* scale_linear_;
    tank_drive_msg.right_speed = joy_msg->axes[axis_tank_right_]* scale_linear_;
    ROS_ERROR("%f, %f", tank_drive_msg.left_speed, tank_drive_msg.right_speed);
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


void JoystickController::enabledCallback(const std_msgs::Bool::ConstPtr& bool_msg)
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
}

}

