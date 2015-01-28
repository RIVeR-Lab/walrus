#include "walrus_joystick_controller/joystick_controller.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"

namespace walrus_joystick_controller {


JoystickController::JoystickController(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  pnh.param<int>("axis_linear", axis_linear_, 1);
  pnh.param<double>("scale_linear", scale_linear_, 0.5);

  pnh.param<int>("axis_angular", axis_angular_, 0);
  pnh.param<double>("scale_angular", scale_angular_, 1.0);

  pnh.param<int>("button_pods_up", button_pods_up_, 0);
  pnh.param<int>("button_pods_toes", button_pods_toes_, 1);
  pnh.param<int>("button_pods_flat", button_pods_flat_, 2);

  joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 1, &JoystickController::joyCallback, this);
  enabled_sub_ = nh.subscribe<std_msgs::Bool>("enabled", 1, &JoystickController::enabledCallback, this);

  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  back_left_pod_pub_ = nh.advertise<std_msgs::Float64>("/walrus/back_left_pod_joint_controller/command", 1);
  back_right_pod_pub_ = nh.advertise<std_msgs::Float64>("/walrus/back_right_pod_joint_controller/command", 1);
  front_left_pod_pub_ = nh.advertise<std_msgs::Float64>("/walrus/front_left_pod_joint_controller/command", 1);
  front_right_pod_pub_ = nh.advertise<std_msgs::Float64>("/walrus/front_right_pod_joint_controller/command", 1);
}

void JoystickController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  if(enabled_)
  {
    geometry_msgs::Twist twist_msg;

    twist_msg.linear.x = joy_msg->axes[axis_linear_] * scale_linear_;
    twist_msg.angular.z = joy_msg->axes[axis_angular_] * scale_angular_;
    cmd_vel_pub_.publish(twist_msg);

    if(joy_msg->buttons[button_pods_up_]) {
      publishPodAngles(0.3);
    }
    else if(joy_msg->buttons[button_pods_toes_]) {
      publishPodAngles(-0.3);
    }
    else if(joy_msg->buttons[button_pods_flat_]) {
      publishPodAngles(0.0);
    }
  }
}

void JoystickController::publishPodAngles(double angle) {
  std_msgs::Float64 angle_msg;
  angle_msg.data = angle;
  std_msgs::Float64 negated_angle_msg;
  negated_angle_msg.data = -angle;

  back_left_pod_pub_.publish(angle_msg);
  back_right_pod_pub_.publish(angle_msg);

  front_left_pod_pub_.publish(negated_angle_msg);
  front_right_pod_pub_.publish(negated_angle_msg);
}


void JoystickController::enabledCallback(const std_msgs::Bool::ConstPtr& bool_msg)
{
  enabled_ = bool_msg->data;

  if(enabled_)
  {
    // Publish zero twist
    geometry_msgs::Twist twist_msg;
    cmd_vel_pub_.publish(twist_msg);
  }
}

}

