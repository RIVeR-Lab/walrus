#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/tfMessage.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <control_toolbox/pid.h>
#include <control_msgs/JointControllerState.h>
#include <boost/scoped_ptr.hpp>
#include <urdf/model.h>
#include <std_msgs/Float64.h>

namespace walrus_pod_controller{

class Pod{
public:
  Pod(ros::NodeHandle nh);
  bool init(hardware_interface::EffortJointInterface* hw, urdf::Model urdf);
  void update(const ros::Time& time, const ros::Duration& period);
  void starting(const ros::Time& time);
  void stopping(const ros::Time& time);

private:
  void setCommandCallback(const std_msgs::Float64ConstPtr& msg);

private:
  ros::NodeHandle nh_;
  hardware_interface::JointHandle joint_;
  boost::shared_ptr<const urdf::Joint> joint_urdf_;
  realtime_tools::RealtimeBuffer<double> command_buffer_;
  control_toolbox::Pid pid_controller_;
  boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
      control_msgs::JointControllerState> > controller_state_publisher_ ;

  ros::Subscriber command_sub_;
  ros::Time next_state_update_;
  ros::Duration controller_state_period_;
};

class WalrusPodController
  : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
 public:
  WalrusPodController();

  bool init(hardware_interface::EffortJointInterface* hw,
	    ros::NodeHandle& root_nh,
	    ros::NodeHandle &controller_nh);

  void update(const ros::Time& time, const ros::Duration& period);

  void starting(const ros::Time& time);

  void stopping(const ros::Time& time);

 private:
  boost::scoped_ptr<Pod> front;
  boost::scoped_ptr<Pod> back;

};

PLUGINLIB_EXPORT_CLASS(walrus_pod_controller::WalrusPodController, controller_interface::ControllerBase);
} // namespace walrus_pod_controller
