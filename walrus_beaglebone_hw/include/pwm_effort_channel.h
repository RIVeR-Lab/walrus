#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <boost/shared_ptr.hpp>


class PWMEffortChannelBase{
 public:
  PWMEffortChannelBase(std::string name,
			   hardware_interface::JointStateInterface& jsi,
		       hardware_interface::EffortJointInterface& jei):
  cmd(0), pos(0), vel(0), eff(0){
 
    hardware_interface::JointStateHandle state_handle(name, &pos, &vel, &eff);
    jsi.registerHandle(state_handle);

    hardware_interface::JointHandle effort_handle(state_handle, &cmd);
    jei.registerHandle(effort_handle);
  }

  virtual void read() = 0;

  void write(){
    //TODO actually write to device
  }

 protected:
  double cmd;
  double pos;
  double vel;
  double eff;
};


class PWMPositionEffortChannel : public PWMEffortChannelBase{
 public:
  PWMPositionEffortChannel(std::string name,
			   hardware_interface::JointStateInterface& jsi,
			       hardware_interface::EffortJointInterface& jei):
  PWMEffortChannelBase(name, jsi, jei)
{ }

	virtual void read(){
	}
};

typedef boost::shared_ptr<PWMPositionEffortChannel> PWMPositionEffortChannelPtr;


class PWMVelocityEffortChannel : public PWMEffortChannelBase{
 public:
  PWMVelocityEffortChannel(std::string name,
			   hardware_interface::JointStateInterface& jsi,
			       hardware_interface::EffortJointInterface& jei):
  PWMEffortChannelBase(name, jsi, jei)
{ }

	virtual void read(){
	}
};


typedef boost::shared_ptr<PWMVelocityEffortChannel> PWMVelocityEffortChannelPtr;
