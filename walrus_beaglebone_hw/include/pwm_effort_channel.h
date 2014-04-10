#include <hardware_interface/robot_hw.h>
#include <boost/shared_ptr.hpp>
#include <transmission_interface/transmission_interface.h>
#include <river_ros_util/ros_control_util.h>

using namespace transmission_interface;
class PWMEffortChannelBase : public river_ros_util::RobotHWActuator{
 public:
  PWMEffortChannelBase(std::string name,
			   hardware_interface::ActuatorStateInterface& jsi,
		       hardware_interface::EffortActuatorInterface& jei):
  cmd(0), pos(0), vel(0), eff(0), name(name){
 
    hardware_interface::ActuatorStateHandle state_handle(name, &pos, &vel, &eff);
    jsi.registerHandle(state_handle);

    hardware_interface::ActuatorHandle effort_handle(state_handle, &cmd);
    jei.registerHandle(effort_handle);
  }

  virtual void read() = 0;

  void write(){
    ROS_INFO_STREAM_THROTTLE(5, "output[" << name<<"] = "<<cmd);
    eff = cmd;
    //TODO actually write to device
  }

  virtual ActuatorData transmissionData(){
    ActuatorData data;
    data.position.push_back(&pos);
    data.velocity.push_back(&vel);
    data.effort.push_back(&eff);
    return data;
  }

 protected:
  double cmd;
  double pos;
  double vel;
  double eff;
  std::string name;
};
typedef boost::shared_ptr<PWMEffortChannelBase> PWMEffortChannelBasePtr;


class PWMPositionEffortChannel : public PWMEffortChannelBase{
 public:
  PWMPositionEffortChannel(std::string name,
			   hardware_interface::ActuatorStateInterface& jsi,
			       hardware_interface::EffortActuatorInterface& jei):
  PWMEffortChannelBase(name, jsi, jei)
{ }

	virtual void read(){
	}
};

typedef boost::shared_ptr<PWMPositionEffortChannel> PWMPositionEffortChannelPtr;


class PWMVelocityEffortChannel : public PWMEffortChannelBase{
 public:
  PWMVelocityEffortChannel(std::string name,
			   hardware_interface::ActuatorStateInterface& jsi,
			       hardware_interface::EffortActuatorInterface& jei):
  PWMEffortChannelBase(name, jsi, jei)
{ }

	virtual void read(){
	}
};


typedef boost::shared_ptr<PWMVelocityEffortChannel> PWMVelocityEffortChannelPtr;
