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
#include <position_effort_controller/PositionEffortCommandStamped.h>

namespace position_effort_controller{

class PositionEffortController
  : public controller_interface::Controller<hardware_interface::EffortJointInterface>{
public:
  PositionEffortController();
  bool init(hardware_interface::EffortJointInterface* hw,
	    ros::NodeHandle& nh,
	    ros::NodeHandle& pnh);
  void update(const ros::Time& time, const ros::Duration& period);
  void starting(const ros::Time& time);
  void stopping(const ros::Time& time);

private:
  void setCommandCallback(const position_effort_controller::PositionEffortCommandConstPtr& msg);

private:
  hardware_interface::JointHandle joint_;
  boost::shared_ptr<const urdf::Joint> joint_urdf_;
  realtime_tools::RealtimeBuffer<position_effort_controller::PositionEffortCommandStamped> command_buffer_;
  control_toolbox::Pid pid_controller_;
  boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
      control_msgs::JointControllerState> > controller_state_publisher_ ;

  ros::Subscriber command_sub_;
  ros::Time next_state_update_;
  ros::Duration controller_state_period_;
  position_effort_controller::PositionEffortCommand last_command_;
  double command_timeout_;
};


PLUGINLIB_EXPORT_CLASS(position_effort_controller::PositionEffortController, controller_interface::ControllerBase);
} // namespace position_effort_controller
