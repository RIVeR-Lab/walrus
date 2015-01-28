#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/tfMessage.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <walrus_drive_controller/odometry.h>
#include <walrus_drive_controller/speed_limiter.h>

namespace walrus_drive_controller{

class WalrusDriveController
  : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
 public:
  WalrusDriveController();

  bool init(hardware_interface::VelocityJointInterface* hw,
	    ros::NodeHandle& root_nh,
	    ros::NodeHandle &controller_nh);

  void update(const ros::Time& time, const ros::Duration& period);

  void starting(const ros::Time& time);

  void stopping(const ros::Time& time);

 private:
  std::string name_;


  // parameters
  ros::Duration publish_period_;
  ros::Time last_state_publish_time_;
  double cmd_vel_timeout_;

  double main_tread_separation_;
  double main_tread_ground_contact_length_;
  double tread_width_;
  double tread_driver_radius_;


  std::string base_frame_id_;
  std::string odom_frame_id_;


  hardware_interface::JointHandle left_tread_joint_;
  hardware_interface::JointHandle right_tread_joint_;


  Odometry odometry_;

  boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
  boost::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;

  realtime_tools::RealtimeBuffer<geometry_msgs::TwistStamped> cmd_vel_buffer_;
  ros::Subscriber cmd_vel_sub_;



  // Speed limiters:
  geometry_msgs::Twist last_cmd_vel_;
  SpeedLimiter limiter_lin_;
  SpeedLimiter limiter_ang_;

 private:
  void brake();

  void cmdVelCallback(const geometry_msgs::Twist& command);

  void setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

};

PLUGINLIB_EXPORT_CLASS(walrus_drive_controller::WalrusDriveController, controller_interface::ControllerBase);
} // namespace walrus_drive_controller
