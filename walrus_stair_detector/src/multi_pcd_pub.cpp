#include <ros/ros.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/publisher.h"


int main (int argc, char** argv) {
  ros::init (argc, argv, "multi");

  std::vector<sensor_msgs::PointCloud2> clouds;
  clouds.resize(argc - 1);
  for(int i = 1; i < argc; ++i) {
    pcl::io::loadPCDFile(argv[i], clouds[i-1]);
    ROS_INFO("Loaded: %s", argv[i]);
  }

  ros::NodeHandle nh;
  pcl_ros::Publisher<sensor_msgs::PointCloud2> pub;
  pub.advertise(nh, "/cloud_pcd", 1);

  while(ros::ok()) {
    for(int i = 0; i < clouds.size(); ++i) {
      clouds[i].header.stamp = ros::Time::now();
      pub.publish(clouds[i]);
      ros::Duration(2.0).sleep();
      ros::spinOnce();
    }
  }

  return 0;
}
