#include <ros/ros.h>
#include "multi_usb_cam/multi_usb_cam.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multi_usb_cam");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  multi_usb_cam::MultiUsbCamNode multi_cam(nh, pnh);

  multi_cam.spin();

  return 0;
}
