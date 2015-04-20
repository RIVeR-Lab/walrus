#include <walrus_stair_detector/walrus_stair_detector_ros.h>
#include <walrus_stair_detector/Stair.h>
#include <pcl_conversions/pcl_conversions.h>

namespace walrus_stair_detector {

WalrusStairDetectorRos::WalrusStairDetectorRos(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh) {
  points_sub_ = nh_.subscribe<PointCloud>("points", 1, &WalrusStairDetectorRos::pointsCallback, this);
  stair_pub_ = nh_.advertise<Stair>("stairs", 1);
}

static geometry_msgs::Point toPointMsg(Eigen::Vector3f v) {
  geometry_msgs::Point m;
  m.x = v[0];
  m.y = v[1];
  m.z = v[2];
  return m;
}
static geometry_msgs::Vector3 toVector3Msg(Eigen::Vector3f v) {
  geometry_msgs::Vector3 m;
  m.x = v[0];
  m.y = v[1];
  m.z = v[2];
  return m;
}

void WalrusStairDetectorRos::pointsCallback(const PointCloud::ConstPtr& msg) {
  Eigen::Vector3f vertical;
  double angle = 23.0 / 180 * M_PI; // angle down from horizontal
  vertical << 0, -cos(angle), -sin(angle);

  std::vector<StairModel> stairs;
  MultiTimer timer;
  detector.detect(msg, vertical, &stairs, &timer);
  TimerPeriod time = timer.getTotal();
  if(stairs.size() > 0) {
    ROS_INFO_STREAM("Found " << stairs.size() << " stairs in " << time.wall << "s (User: " << time.user << "s, System: " << time.system << "s)");

    const StairModel& stair_model = stairs[0];
    Stair stair_msg;
    pcl_conversions::fromPCL(msg->header, stair_msg.header);
    stair_msg.origin = toPointMsg(stair_model.origin);
    stair_msg.vertical = toVector3Msg(stair_model.vertical);
    stair_msg.direction = toVector3Msg(stair_model.direction);
    stair_msg.horizontal = toVector3Msg(stair_model.horizontal);
    stair_msg.rise = stair_model.rise;
    stair_msg.run = stair_model.run;
    stair_msg.width = stair_model.width;
    stair_msg.num_stairs = stair_model.num_stairs;
    stair_pub_.publish(stair_msg);
  }
  else {
    ROS_INFO_STREAM("Found no stairs in " << time.wall << "s (User: " << time.user << "s, System: " << time.system << "s)");
  }
}


}
