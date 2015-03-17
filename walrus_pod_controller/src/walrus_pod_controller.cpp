#include <tf/transform_datatypes.h>

#include <urdf_parser/urdf_parser.h>

#include <boost/assign.hpp>
#include <angles/angles.h>
#include <control_toolbox/filters.h>
#include <walrus_pod_controller/walrus_pod_controller.h>

namespace walrus_pod_controller{

Pod::Pod(ros::NodeHandle nh)
  : nh_(nh), next_state_update_(0), controller_state_period_(0.5) {
  last_command_.mode = walrus_pod_controller::PodCommand::DISABLED;
}

bool Pod::init(hardware_interface::EffortJointInterface* hw, urdf::Model urdf) {
  std::string joint_name;
  if (!nh_.getParam("joint", joint_name)) {
    ROS_ERROR("No joint given (namespace: %s)", nh_.getNamespace().c_str());
    return false;
  }

  joint_ = hw->getHandle(joint_name);
  boost::shared_ptr<const urdf::Joint> joint_urdf =  urdf.getJoint(joint_name);
  if (!joint_urdf) {
    ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
    return false;
  }

  if (!pid_controller_.init(ros::NodeHandle(nh_, "pid")))
    return false;

  controller_state_publisher_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(nh_, "state", 1));

  command_sub_ = nh_.subscribe<walrus_pod_controller::PodCommand>("command", 1, &Pod::setCommandCallback, this);
}

void Pod::setCommandCallback(const walrus_pod_controller::PodCommandConstPtr& msg) {
  walrus_pod_controller::PodCommand command = *msg;
  command_buffer_.writeFromNonRT(command);
}

void Pod::starting(const ros::Time& time) {
  walrus_pod_controller::PodCommand command;
  command.mode = walrus_pod_controller::PodCommand::HOLD_POSITION;
  command_buffer_.initRT(command);
}

void Pod::stopping(const ros::Time& time) {
  joint_.setCommand(0.0); // set joint effort to zero
}

void Pod::update(const ros::Time& time, const ros::Duration& period) {
  double current_position = joint_.getPosition();
  walrus_pod_controller::PodCommand command = *(command_buffer_.readFromRT());

  double command_effort;
  double command_position;
  double error;
  if(command.mode == walrus_pod_controller::PodCommand::POSITION ||
     command.mode == walrus_pod_controller::PodCommand::HOLD_POSITION) {
    // switching to position mode so reset the pid controller
    if(last_command_.mode != walrus_pod_controller::PodCommand::POSITION &&
       last_command_.mode != walrus_pod_controller::PodCommand::HOLD_POSITION) {
      pid_controller_.reset();
    }

    if(command.mode == walrus_pod_controller::PodCommand::HOLD_POSITION) {
      // Hold the position that was held last time
      if(last_command_.mode == walrus_pod_controller::PodCommand::HOLD_POSITION) {
	command.set_point = last_command_.set_point;
      }
      else {
	command.set_point = current_position;
      }
    }

    command_position = filters::clamp(command.set_point, -M_PI, M_PI);
    error = angles::shortest_angular_distance(current_position, command_position);
    command_effort = pid_controller_.computeCommand(error, period);
  }
  else if(command.mode == walrus_pod_controller::PodCommand::EFFORT) {
    command_position = std::numeric_limits<double>::quiet_NaN();
    error = std::numeric_limits<double>::quiet_NaN();
    command_effort = command.set_point;
  }
  else {
    command_position = std::numeric_limits<double>::quiet_NaN();
    error = std::numeric_limits<double>::quiet_NaN();
    command_effort = 0.0;
  }
  joint_.setCommand(command_effort);
  last_command_ = command;

  // publish state
  if (time > next_state_update_) {
    if(controller_state_publisher_ && controller_state_publisher_->trylock()) {
      controller_state_publisher_->msg_.header.stamp = time;
      controller_state_publisher_->msg_.set_point = command_position;
      controller_state_publisher_->msg_.process_value = current_position;
      controller_state_publisher_->msg_.process_value_dot = joint_.getVelocity();
      controller_state_publisher_->msg_.error = error;
      controller_state_publisher_->msg_.time_step = period.toSec();
      controller_state_publisher_->msg_.command = command_effort;

      double dummy;
      pid_controller_.getGains(controller_state_publisher_->msg_.p,
			       controller_state_publisher_->msg_.i,
			       controller_state_publisher_->msg_.d,
			       controller_state_publisher_->msg_.i_clamp,
			       dummy);
      controller_state_publisher_->unlockAndPublish();
    }
    next_state_update_ = time + controller_state_period_;
  }
}


WalrusPodController::WalrusPodController()
{
}

bool WalrusPodController::init(hardware_interface::EffortJointInterface* hw,
				 ros::NodeHandle& root_nh,
				 ros::NodeHandle &controller_nh)
{
  urdf::Model urdf;
  if (!urdf.initParam("robot_description")) {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }

  front.reset(new Pod(ros::NodeHandle(controller_nh, "front")));
  if(!front->init(hw, urdf))
    return false;

  back.reset(new Pod(ros::NodeHandle(controller_nh, "back")));
  if(!back->init(hw, urdf))
    return false;


  return true;
}

void WalrusPodController::update(const ros::Time& time, const ros::Duration& period)
{
  front->update(time, period);
  back->update(time, period);
}

void WalrusPodController::starting(const ros::Time& time)
{
  front->starting(time);
  back->starting(time);
}

void WalrusPodController::stopping(const ros::Time& time)
{
  front->stopping(time);
  back->stopping(time);
}



} // namespace diff_pod_controller

