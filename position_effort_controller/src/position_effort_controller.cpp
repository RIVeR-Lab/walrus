#include <tf/transform_datatypes.h>

#include <urdf_parser/urdf_parser.h>

#include <boost/assign.hpp>
#include <angles/angles.h>
#include <control_toolbox/filters.h>
#include <position_effort_controller/position_effort_controller.h>

namespace position_effort_controller{

PositionEffortController::PositionEffortController()
 : next_state_update_(0), controller_state_period_(0.5), command_timeout_(0.5)
{
}

bool PositionEffortController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh, ros::NodeHandle & pnh) 
{	 
  std::string joint_name;
  
  urdf::Model urdf;
  if (!urdf.initParam("robot_description")) {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }
  
  nh.param("command_timeout", command_timeout_, command_timeout_);

  if (!nh.getParam("joint", joint_name)) {
    ROS_ERROR("No joint given (namespace: %s)", nh.getNamespace().c_str());
    return false;
  }

  joint_ = hw->getHandle(joint_name);
  boost::shared_ptr<const urdf::Joint> joint_urdf =  urdf.getJoint(joint_name);
  if (!joint_urdf) {
    ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
    return false;
  }

  if (!pid_controller_.init(ros::NodeHandle(nh, "pid")))
    return false;

  controller_state_publisher_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(nh, "state", 1));

  command_sub_ = nh.subscribe<position_effort_controller::PositionEffortCommand>("command", 1, &PositionEffortController::setCommandCallback, this);
}

void PositionEffortController::setCommandCallback(const position_effort_controller::PositionEffortCommandConstPtr& command) {
  position_effort_controller::PositionEffortCommandStamped command_stamped;
  command_stamped.header.stamp = ros::Time::now();
  command_stamped.command = *command;
  command_buffer_.writeFromNonRT(command_stamped);
}

void PositionEffortController::starting(const ros::Time& time) {
  position_effort_controller::PositionEffortCommandStamped command;
  command.header.stamp = ros::Time(0);
  command.command.mode = position_effort_controller::PositionEffortCommand::HOLD_POSITION;
  command_buffer_.initRT(command);
}

void PositionEffortController::stopping(const ros::Time& time) {
  joint_.setCommand(0.0); // set joint effort to zero
}

void PositionEffortController::update(const ros::Time& time, const ros::Duration& period) {
  double current_position = joint_.getPosition();
  position_effort_controller::PositionEffortCommandStamped command_stamped = *(command_buffer_.readFromRT());

  const double dt = (time - command_stamped.header.stamp).toSec();
  double command_effort;
  double command_position;
  double error;

  position_effort_controller::PositionEffortCommand command = command_stamped.command;
  if (dt > command_timeout_) {
    // timeout
    command.mode = position_effort_controller::PositionEffortCommand::DISABLED;
  }

  if(command.mode == position_effort_controller::PositionEffortCommand::POSITION ||
     command.mode == position_effort_controller::PositionEffortCommand::HOLD_POSITION) {
    // switching to position mode so reset the pid controller
    if(last_command_.mode != position_effort_controller::PositionEffortCommand::POSITION &&
       last_command_.mode != position_effort_controller::PositionEffortCommand::HOLD_POSITION) {
      pid_controller_.reset();
    }

    if(command.mode == position_effort_controller::PositionEffortCommand::HOLD_POSITION) {
      // Hold the position that was held last time
      if(last_command_.mode == position_effort_controller::PositionEffortCommand::HOLD_POSITION) {
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
  else if(command.mode == position_effort_controller::PositionEffortCommand::EFFORT) {
    command_position = std::numeric_limits<double>::quiet_NaN();
    error = std::numeric_limits<double>::quiet_NaN();
    command_effort = command.set_point;
  }
  else { // disabled
    command_position = std::numeric_limits<double>::quiet_NaN();
    error = std::numeric_limits<double>::quiet_NaN();
    command_effort = 0.0;
  }
  last_command_ = command;
  joint_.setCommand(command_effort);

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


} // namespace position_effort_controller

