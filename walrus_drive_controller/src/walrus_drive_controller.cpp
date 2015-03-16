#include <tf/transform_datatypes.h>

#include <urdf_parser/urdf_parser.h>

#include <boost/assign.hpp>

#include <walrus_drive_controller/walrus_drive_controller.h>

static double euclideanOfVectors(const urdf::Vector3& vec1, const urdf::Vector3& vec2)
{
  return std::sqrt(std::pow(vec1.x-vec2.x,2) +
                   std::pow(vec1.y-vec2.y,2) +
                   std::pow(vec1.z-vec2.z,2));
}

namespace walrus_drive_controller{

WalrusDriveController::WalrusDriveController()
  : command_timeout_(0.5)

  , main_tread_separation_(1.0)
  , main_tread_ground_contact_length_(1.0)
  , tread_width_(1.0)
  , tread_driver_radius_(1.0)

  , base_frame_id_("base_link")
  , odom_frame_id_("odom")
{
}

bool WalrusDriveController::init(hardware_interface::VelocityJointInterface* hw,
				 ros::NodeHandle& root_nh,
				 ros::NodeHandle &controller_nh)
{
  // compute name to use for logging
  const std::string complete_ns = controller_nh.getNamespace();
  std::size_t id = complete_ns.find_last_of("/");
  name_ = complete_ns.substr(id + 1);

  // Get joint names
  std::string left_tread_name;
  std::string right_tread_name;
  controller_nh.param("left_tread_joint", left_tread_name, left_tread_name);
  controller_nh.param("right_tread_joint", right_tread_name, right_tread_name);



  // Tread specifications
  controller_nh.param("main_tread_separation", main_tread_separation_, main_tread_separation_);
  ROS_INFO_STREAM_NAMED(name_, "Main tread separation: " << main_tread_separation_);

  controller_nh.param("main_tread_ground_contact_length", main_tread_ground_contact_length_, main_tread_ground_contact_length_);
  ROS_INFO_STREAM_NAMED(name_, "Main tread ground contact length: " << main_tread_ground_contact_length_);

  controller_nh.param("tread_width", tread_width_, tread_width_);
  ROS_INFO_STREAM_NAMED(name_, "Tread width: " << tread_width_);

  controller_nh.param("tread_driver_radius", tread_driver_radius_, tread_driver_radius_);
  ROS_INFO_STREAM_NAMED(name_, "Tread driver radius: " << tread_driver_radius_);

  odometry_.setWheelParams(main_tread_separation_, tread_driver_radius_);


  // Rates and timeouts
  controller_nh.param("command_timeout", command_timeout_, command_timeout_);
  ROS_INFO_STREAM_NAMED(name_, "Velocity commands will be considered old if they are older than "
                        << command_timeout_ << "s.");
  double publish_rate;
  controller_nh.param("publish_rate", publish_rate, 50.0);
  ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at "
                        << publish_rate << "Hz.");
  publish_period_ = ros::Duration(1.0 / publish_rate);


  // TF Frames
  controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
  ROS_INFO_STREAM_NAMED(name_, "Base frame_id set to " << base_frame_id_);

  controller_nh.param("odom_frame_id", odom_frame_id_, odom_frame_id_);
  ROS_INFO_STREAM_NAMED(name_, "Odom frame_id set to " << odom_frame_id_);


  // Velocity and acceleration limits:
  controller_nh.param("linear/x/has_velocity_limits"    , limiter_lin_.has_velocity_limits    , limiter_lin_.has_velocity_limits    );
  controller_nh.param("linear/x/has_acceleration_limits", limiter_lin_.has_acceleration_limits, limiter_lin_.has_acceleration_limits);
  controller_nh.param("linear/x/max_velocity"           , limiter_lin_.max_velocity           ,  limiter_lin_.max_velocity          );
  controller_nh.param("linear/x/min_velocity"           , limiter_lin_.min_velocity           , -limiter_lin_.max_velocity          );
  controller_nh.param("linear/x/max_acceleration"       , limiter_lin_.max_acceleration       ,  limiter_lin_.max_acceleration      );
  controller_nh.param("linear/x/min_acceleration"       , limiter_lin_.min_acceleration       , -limiter_lin_.max_acceleration      );

  controller_nh.param("angular/z/has_velocity_limits"    , limiter_ang_.has_velocity_limits    , limiter_ang_.has_velocity_limits    );
  controller_nh.param("angular/z/has_acceleration_limits", limiter_ang_.has_acceleration_limits, limiter_ang_.has_acceleration_limits);
  controller_nh.param("angular/z/max_velocity"           , limiter_ang_.max_velocity           ,  limiter_ang_.max_velocity          );
  controller_nh.param("angular/z/min_velocity"           , limiter_ang_.min_velocity           , -limiter_ang_.max_velocity          );
  controller_nh.param("angular/z/max_acceleration"       , limiter_ang_.max_acceleration       ,  limiter_ang_.max_acceleration      );
  controller_nh.param("angular/z/min_acceleration"       , limiter_ang_.min_acceleration       , -limiter_ang_.max_acceleration      );

  setOdomPubFields(root_nh, controller_nh);

  left_tread_joint_ = hw->getHandle(left_tread_name);
  right_tread_joint_ = hw->getHandle(right_tread_name);


  cmd_vel_sub_ = controller_nh.subscribe("cmd_vel", 1, &WalrusDriveController::cmdVelCallback, this);
  tank_drive_sub_ = controller_nh.subscribe("tank_drive", 1, &WalrusDriveController::tankCallback, this);

  return true;
}

void WalrusDriveController::update(const ros::Time& time, const ros::Duration& period)
{

  double left_pos  = left_tread_joint_.getPosition();
  double right_pos = right_tread_joint_.getPosition();
  odometry_.update(left_pos, right_pos, time);

  // Publish odometry message
  if(last_state_publish_time_ + publish_period_ < time)
  {
    last_state_publish_time_ += publish_period_;
    // Compute and store orientation info
    const geometry_msgs::Quaternion orientation(
						  tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

    // Populate odom message and publish
    if(odom_pub_->trylock())
    {
      odom_pub_->msg_.header.stamp = time;
      odom_pub_->msg_.pose.pose.position.x = odometry_.getX();
      odom_pub_->msg_.pose.pose.position.y = odometry_.getY();
      odom_pub_->msg_.pose.pose.orientation = orientation;
      odom_pub_->msg_.twist.twist.linear.x  = odometry_.getLinear();
      odom_pub_->msg_.twist.twist.angular.z = odometry_.getAngular();
      odom_pub_->unlockAndPublish();
    }

    // Publish tf /odom frame
    if (tf_odom_pub_->trylock())
    {
      geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
      odom_frame.header.stamp = time;
      odom_frame.transform.translation.x = odometry_.getX();
      odom_frame.transform.translation.y = odometry_.getY();
      odom_frame.transform.rotation = orientation;
      tf_odom_pub_->unlockAndPublish();
    }
  }

  // MOVE ROBOT
  // Retreive current velocity command and time step:
  geometry_msgs::TwistStamped cmd_vel_stamped = *(cmd_vel_buffer_.readFromRT());
  walrus_drive_controller::TankDriveCommandStamped tank_drive_stamped = *(tank_drive_buffer_.readFromRT());

  // twist command in newer
  if(cmd_vel_stamped.header.stamp > tank_drive_stamped.header.stamp) {
    geometry_msgs::Twist curr_cmd = cmd_vel_stamped.twist;
    const double dt = (time - cmd_vel_stamped.header.stamp).toSec();

    // Brake if cmd_vel has timeout:
    if (dt > command_timeout_) {
      brake();
    }
    else {
      // Limit velocities and accelerations:
      const double cmd_dt(period.toSec());
      limiter_lin_.limit(curr_cmd.linear.x, odometry_.getLinear(), cmd_dt);
      limiter_ang_.limit(curr_cmd.angular.z, odometry_.getAngular(), cmd_dt);

      const double vel_left  = (curr_cmd.linear.x - curr_cmd.angular.z * main_tread_separation_ / 2.0) / tread_driver_radius_;
      const double vel_right = (curr_cmd.linear.x + curr_cmd.angular.z * main_tread_separation_ / 2.0) / tread_driver_radius_;

      left_tread_joint_.setCommand(vel_left);
      right_tread_joint_.setCommand(vel_right);
    }
  }
  else {
    walrus_drive_controller::TankDriveCommand curr_cmd = tank_drive_stamped.command;
    const double dt = (time - tank_drive_stamped.header.stamp).toSec();

    if (dt > command_timeout_) {
      brake();
    }
    else {
      left_tread_joint_.setCommand(curr_cmd.left_speed/tread_driver_radius_);
      right_tread_joint_.setCommand(curr_cmd.right_speed/tread_driver_radius_);
    }
  }


}

void WalrusDriveController::starting(const ros::Time& time)
{
  brake();

  // Register starting time used to keep fixed rate
  last_state_publish_time_ = time;

  odometry_.init(time);
  geometry_msgs::TwistStamped twist;
  twist.header.stamp = ros::Time(0);
  twist.twist.linear.x = 0;
  twist.twist.angular.z = 0;
  cmd_vel_buffer_.initRT(twist);
  walrus_drive_controller::TankDriveCommandStamped tank_drive;
  tank_drive.header.stamp = ros::Time(0);
  tank_drive.command.left_speed = 0;
  tank_drive.command.right_speed = 0;
  tank_drive_buffer_.initRT(tank_drive);
}

void WalrusDriveController::stopping(const ros::Time& time)
{
  brake();
}

void WalrusDriveController::brake()
{
  left_tread_joint_.setCommand(0.0);
  right_tread_joint_.setCommand(0.0);
}

void WalrusDriveController::cmdVelCallback(const geometry_msgs::Twist& command)
{
  if(isRunning())
  {
    geometry_msgs::TwistStamped command_stamped;
    command_stamped.header.stamp = ros::Time::now();
    command_stamped.twist = command;
    cmd_vel_buffer_.writeFromNonRT(command_stamped);
  }
  else
  {
    ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
  }
}
void WalrusDriveController::tankCallback(const walrus_drive_controller::TankDriveCommand& command)
{
  if(isRunning())
  {
    walrus_drive_controller::TankDriveCommandStamped command_stamped;
    command_stamped.header.stamp = ros::Time::now();
    command_stamped.command = command;
    tank_drive_buffer_.writeFromNonRT(command_stamped);
  }
  else
  {
    ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
  }
}


void WalrusDriveController::setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  // Get and check params for covariances
  XmlRpc::XmlRpcValue pose_cov_list;
  controller_nh.getParam("pose_covariance_diagonal", pose_cov_list);
  ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(pose_cov_list.size() == 6);
  for (int i = 0; i < pose_cov_list.size(); ++i)
    ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

  XmlRpc::XmlRpcValue twist_cov_list;
  controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
  ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(twist_cov_list.size() == 6);
  for (int i = 0; i < twist_cov_list.size(); ++i)
    ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

  // Setup odometry realtime publisher + odom message constant fields
  odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
  odom_pub_->msg_.header.frame_id = odom_frame_id_;
  odom_pub_->msg_.child_frame_id = base_frame_id_;
  odom_pub_->msg_.pose.pose.position.z = 0;
  odom_pub_->msg_.pose.covariance = boost::assign::list_of
    (static_cast<double>(pose_cov_list[0])) (0)  (0)  (0)  (0)  (0)
    (0)  (static_cast<double>(pose_cov_list[1])) (0)  (0)  (0)  (0)
    (0)  (0)  (static_cast<double>(pose_cov_list[2])) (0)  (0)  (0)
    (0)  (0)  (0)  (static_cast<double>(pose_cov_list[3])) (0)  (0)
    (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[4])) (0)
    (0)  (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[5]));
  odom_pub_->msg_.twist.twist.linear.y  = 0;
  odom_pub_->msg_.twist.twist.linear.z  = 0;
  odom_pub_->msg_.twist.twist.angular.x = 0;
  odom_pub_->msg_.twist.twist.angular.y = 0;
  odom_pub_->msg_.twist.covariance = boost::assign::list_of
    (static_cast<double>(twist_cov_list[0])) (0)  (0)  (0)  (0)  (0)
    (0)  (static_cast<double>(twist_cov_list[1])) (0)  (0)  (0)  (0)
    (0)  (0)  (static_cast<double>(twist_cov_list[2])) (0)  (0)  (0)
    (0)  (0)  (0)  (static_cast<double>(twist_cov_list[3])) (0)  (0)
    (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[4])) (0)
    (0)  (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[5]));
  tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
  tf_odom_pub_->msg_.transforms.resize(1);
  tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
  tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
  tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_id_;
}

} // namespace diff_drive_controller

