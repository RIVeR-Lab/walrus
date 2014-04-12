#ifndef WALRUS_BASE_HW_H_
#define WALRUS_BASE_HW_H_

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/transmission_info.h>
#include <transmission_interface/transmission_parser.h>
#include <boost/shared_ptr.hpp>
#include "pwm_effort_channel.h"
#include <river_ros_util/ros_util.h>
#include <river_ros_util/ros_control_util.h>
#include <device_driver_base/serial_port.h>
#include "walrus_teensy_interface.h"

using namespace transmission_interface;
using namespace device_driver;

class WalrusBaseRobot : public river_ros_util::AbstractRobotHW
{
public:
 WalrusBaseRobot(ros::NodeHandle n = ros::NodeHandle(), std::string robot_ns="walrus/"):
    teensy(new WalrusTeensyInterface(10)){
    teensy->open();

    add_actuator(PWMPositionEffortChannelPtr(new PWMPositionEffortChannel(robot_ns+"front_left_pod_joint_actuator", 0, 10, as_interface, ae_interface, teensy)));
    add_actuator(PWMPositionEffortChannelPtr(new PWMPositionEffortChannel(robot_ns+"back_left_pod_joint_actuator", 1, 10, as_interface, ae_interface, teensy)));
    add_actuator(PWMPositionEffortChannelPtr(new PWMPositionEffortChannel(robot_ns+"front_right_pod_joint_actuator", 2, 10, as_interface, ae_interface, teensy)));
    add_actuator(PWMPositionEffortChannelPtr(new PWMPositionEffortChannel(robot_ns+"back_right_pod_joint_actuator", 3, 10, as_interface, ae_interface, teensy)));

    add_actuator(PWMVelocityEffortChannelPtr(new PWMVelocityEffortChannel(robot_ns+"left_drive_actuator", 4, 10, as_interface, ae_interface, teensy)));
    add_actuator(PWMVelocityEffortChannelPtr(new PWMVelocityEffortChannel(robot_ns+"right_drive_actuator", 5, 10, as_interface, ae_interface, teensy)));

    std::vector<transmission_interface::TransmissionInfo> transmissions;
    std::string urdf_string = river_ros_util::wait_for_param(n, "robot_description");
    TransmissionParser::parse(urdf_string, transmissions);
    build_transmissions(transmissions);

    register_interfaces();
  }

 private:
  WalrusTeensyInterfacePtr teensy;
  
};

#endif
