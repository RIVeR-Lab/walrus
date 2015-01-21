#include "ros/ros.h"
#include "walrus_joystick_controller/joystick_controller.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  walrus_joystick_controller::JoystickController controller(nh, pnh);

  ros::spin();
}
