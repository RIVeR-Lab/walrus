#include <walrus_stair_detector/walrus_stair_detector_ros.h>

namespace walrus_stair_detector {

WalrusStairDetectorRos::WalrusStairDetectorRos(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh) {
  points_sub_ = nh_.subscribe<PointCloud>("points", 1, &WalrusStairDetectorRos::pointsCallback, this);
}

void WalrusStairDetectorRos::pointsCallback(const PointCloud::ConstPtr& msg) {
  Eigen::Vector3f vertical;
  vertical << 0, -1, 0;

  std::vector<StairModel> stairs;
  walrus_stair_detector::MultiTimer timer;
  detector.detect(msg, vertical, &stairs, &timer);
  if(stairs.size() > 0) {
    ROS_INFO_STREAM("Found " << stairs.size() << " stairs in " << boost::timer::format(timer.getTotal(), 3, "%ws (User: %us, System: %ss)"));
  }
  else {
    ROS_INFO_STREAM("Found no stairs in " << boost::timer::format(timer.getTotal(), 3, "%ws (User: %us, System: %ss)"));
  }
}


}
