#ifndef WALRUS_STAIR_DETECTOR_WALRUS_STAIR_DETECTOR_ROS_H_
#define WALRUS_STAIR_DETECTOR_WALRUS_STAIR_DETECTOR_ROS_H_

#include <ros/ros.h>
#include <walrus_stair_detector/walrus_stair_detector.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

namespace walrus_stair_detector {

class WalrusStairDetectorRos {
public:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

  WalrusStairDetectorRos(ros::NodeHandle& nh, ros::NodeHandle& pnh);
private:
  void pointsCallback(const PointCloud::ConstPtr& msg);


private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  WalrusStairDetector detector;

  ros::Subscriber points_sub_;
  ros::Publisher stair_pub_;
};

}

#endif
