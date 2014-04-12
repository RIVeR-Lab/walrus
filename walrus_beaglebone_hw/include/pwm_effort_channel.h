#ifndef PWM_EFFORT_CHANNEL_H_
#define PWM_EFFORT_CHANNEL_H_

#include <hardware_interface/robot_hw.h>
#include <boost/shared_ptr.hpp>
#include <transmission_interface/transmission_interface.h>
#include <river_ros_util/ros_control_util.h>
#include <device_driver_base/serial_port.h>
#include "walrus_teensy_interface.h"
#include <algorithm>
#include <math.h>

using namespace transmission_interface;
using namespace device_driver;

static const double us_per_torque = 10;
static const double ticks_per_rotation = 10000;

class PWMEffortChannelBase : public river_ros_util::RobotHWComponent{
 public:
   PWMEffortChannelBase(std::string name, uint8_t channel, double max_effort,
			   hardware_interface::ActuatorStateInterface& jsi,
		       hardware_interface::EffortActuatorInterface& jei, WalrusTeensyInterfacePtr& teensy):
  cmd(0), pos(0), vel(0), eff(0), name(name), channel(channel), teensy(teensy), max_effort(max_effort){
    ROS_INFO_STREAM("CREATED: "<<name);
    printf("created\n");

    hardware_interface::ActuatorStateHandle state_handle(name, &pos, &vel, &eff);
    jsi.registerHandle(state_handle);

    hardware_interface::ActuatorHandle effort_handle(state_handle, &cmd);
    jei.registerHandle(effort_handle);
  }

  virtual void read() = 0;

  void write(){
    ROS_INFO_THROTTLE(1, "HI");
    double actual_cmd = std::min(max_effort, std::max(cmd, -max_effort));
    uint16_t period_us = 1500 + us_per_torque*actual_cmd;
    teensy->set_pwm(channel, period_us);
    
    eff = actual_cmd;
    ROS_INFO_THROTTLE(1, "BYE");
  }

 protected:
  double cmd;
  double pos;
  double vel;
  double eff;
  std::string name;
  uint8_t channel;
  double max_effort;
  WalrusTeensyInterfacePtr teensy;
};
typedef boost::shared_ptr<PWMEffortChannelBase> PWMEffortChannelBasePtr;


class PWMPositionEffortChannel : public PWMEffortChannelBase{
 public:
  PWMPositionEffortChannel(std::string name, uint8_t channel, double max_effort,
			   hardware_interface::ActuatorStateInterface& jsi,
			   hardware_interface::EffortActuatorInterface& jei,
                           WalrusTeensyInterfacePtr& teensy):
  PWMEffortChannelBase(name, channel, max_effort, jsi, jei, teensy)
{}

	virtual void read(){
          uint16_t raw_position = teensy->get_analog(channel);
          pos =  2*M_PI * raw_position / 1023;
	}
};

typedef boost::shared_ptr<PWMPositionEffortChannel> PWMPositionEffortChannelPtr;


class PWMVelocityEffortChannel : public PWMEffortChannelBase{
 public:
  PWMVelocityEffortChannel(std::string name, uint8_t channel, double max_effort,
			   hardware_interface::ActuatorStateInterface& jsi,
			   hardware_interface::EffortActuatorInterface& jei,
                           WalrusTeensyInterfacePtr& teensy):
  PWMEffortChannelBase(name, channel, max_effort, jsi, jei, teensy)
{}

	virtual void read(){
          uint16_t raw_position = teensy->get_position(channel);
          uint16_t raw_velocity = teensy->get_velocity(channel);
          pos =  2*M_PI * raw_position / ticks_per_rotation;
          vel =  2*M_PI * raw_velocity / ticks_per_rotation;
	}
};


typedef boost::shared_ptr<PWMVelocityEffortChannel> PWMVelocityEffortChannelPtr;

#endif
