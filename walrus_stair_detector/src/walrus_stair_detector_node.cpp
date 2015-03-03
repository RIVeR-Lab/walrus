#include <ros/ros.h>
#include <walrus_stair_detector/walrus_stair_detector_ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "walrus_stair_detector");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  walrus_stair_detector::WalrusStairDetectorRos detector(nh, pnh);
  ros::spin();
}
