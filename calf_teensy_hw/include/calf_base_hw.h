#ifndef CALF_BASE_HW_H_
#define CALF_BASE_HW_H_

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_info.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <transmission_interface/transmission_parser.h>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include "pwm_effort_channel.h"
#include <river_ros_util/ros_util.h>
#include <river_ros_util/ros_control_util.h>
#include <device_driver_base/serial_port.h>
#include "calf_teensy_interface.h"

using namespace transmission_interface;
using namespace device_driver;

class CalfBaseRobot : public hardware_interface::RobotHW
{
 public:
 CalfBaseRobot(ros::NodeHandle nh = ros::NodeHandle(), std::string robot_ns="calf/"):
  teensy(new CalfTeensyInterface(10)), nh_(nh), robot_ns_(robot_ns){
    teensy->open();
  }

  bool init() {
    add_actuator(PWMPositionEffortChannelPtr(new PWMPositionEffortChannel(robot_ns_+"front_left_pod_joint_actuator", 24, 42, 10, as_interface_, ae_interface_, teensy)));
    add_actuator(PWMPositionEffortChannelPtr(new PWMPositionEffortChannel(robot_ns_+"back_left_pod_joint_actuator", 15, 45, 10, as_interface_, ae_interface_, teensy)));
    add_actuator(PWMPositionEffortChannelPtr(new PWMPositionEffortChannel(robot_ns_+"front_right_pod_joint_actuator", 14, 43, 10, as_interface_, ae_interface_, teensy)));
    add_actuator(PWMPositionEffortChannelPtr(new PWMPositionEffortChannel(robot_ns_+"back_right_pod_joint_actuator", 16, 44, 10, as_interface_, ae_interface_, teensy)));

    add_actuator(PWMVelocityEffortChannelPtr(new PWMVelocityEffortChannel(robot_ns_+"left_drive_actuator", 26, 4, 10, as_interface_, ae_interface_, teensy)));
    add_actuator(PWMVelocityEffortChannelPtr(new PWMVelocityEffortChannel(robot_ns_+"right_drive_actuator", 25, 5, 10, as_interface_, ae_interface_, teensy)));

    registerInterface(&as_interface_);
    registerInterface(&ae_interface_);


    std::string urdf_string = river_ros_util::wait_for_param(nh_, "robot_description");
    if (!transmission_loader_->load(urdf_string)) {return false;}
  }

  void write(){
    robot_transmissions_.get<transmission_interface::JointToActuatorEffortInterface>()->propagate();
    for(unsigned int i = 0; i<actuators.size(); ++i){
      actuators[i]->write();
    }
  }
  void read(){
    for(unsigned int i = 0; i<actuators.size(); ++i){
      actuators[i]->read();
    }
    robot_transmissions_.get<transmission_interface::ActuatorToJointStateInterface>()->propagate();
  }


 private:
  void add_actuator(river_ros_util::RobotHWComponentPtr component){
    actuators.push_back(component);
  }

  CalfTeensyInterfacePtr teensy;
  ros::NodeHandle nh_;
  std::string robot_ns_;
  transmission_interface::RobotTransmissions robot_transmissions_;
  boost::scoped_ptr<transmission_interface::TransmissionInterfaceLoader> transmission_loader_;
  std::vector<river_ros_util::RobotHWComponentPtr> actuators;
  hardware_interface::ActuatorStateInterface as_interface_;
  hardware_interface::EffortActuatorInterface ae_interface_;
};

#endif
