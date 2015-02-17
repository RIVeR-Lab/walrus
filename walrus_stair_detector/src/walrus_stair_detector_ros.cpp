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
  detector.detect(msg, vertical, &stairs);
}


}
