#include "walrus_joystick_controller/joystick_controller.h"
#include "geometry_msgs/Twist.h"
#include <walrus_pod_controller/PodCommand.h>

namespace walrus_joystick_controller {


JoystickController::JoystickController(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  pnh.param<bool>("enable_by_default", enabled_, 1);

  pnh.param<int>("axis_linear", axis_linear_, 1);
  pnh.param<double>("scale_linear", scale_linear_, 0.5);

  pnh.param<int>("axis_angular", axis_angular_, 0);
  pnh.param<double>("scale_angular", scale_angular_, 1.0);

  pnh.param<int>("button_front_pods_up", button_front_pods_up_, 0);
  pnh.param<int>("button_front_pods_down", button_front_pods_down_, 1);
  pnh.param<int>("button_back_pods_up", button_back_pods_up_, 0);
  pnh.param<int>("button_back_pods_down", button_back_pods_down_, 1);

  joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 1, &JoystickController::joyCallback, this);
  enabled_sub_ = nh.subscribe<std_msgs::Bool>("enabled", 1, &JoystickController::enabledCallback, this);

  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  back_left_pod_pub_ = nh.advertise<walrus_pod_controller::PodCommand>("left_pods_joint_controller/back/command", 1);
  back_right_pod_pub_ = nh.advertise<walrus_pod_controller::PodCommand>("right_pods_joint_controller/back/command", 1);
  front_left_pod_pub_ = nh.advertise<walrus_pod_controller::PodCommand>("left_pods_joint_controller/front/command", 1);
  front_right_pod_pub_ = nh.advertise<walrus_pod_controller::PodCommand>("right_pods_joint_controller/front/command", 1);
}

void JoystickController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  if(enabled_)
  {
    geometry_msgs::Twist twist_msg;

    twist_msg.linear.x = joy_msg->axes[axis_linear_] * scale_linear_;
    twist_msg.angular.z = joy_msg->axes[axis_angular_] * scale_angular_;
    cmd_vel_pub_.publish(twist_msg);

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

  if(enabled_)
  {
    // Publish zero twist
    geometry_msgs::Twist twist_msg;
    cmd_vel_pub_.publish(twist_msg);

    walrus_pod_controller::PodCommand hold_msg;
    hold_msg.mode = walrus_pod_controller::PodCommand::HOLD_POSITION;
    back_left_pod_pub_.publish(hold_msg);
    back_right_pod_pub_.publish(hold_msg);
    front_left_pod_pub_.publish(hold_msg);
    front_right_pod_pub_.publish(hold_msg);
  }
}

}

