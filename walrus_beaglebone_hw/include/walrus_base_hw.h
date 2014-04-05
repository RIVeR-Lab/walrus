#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <boost/shared_ptr.hpp>
#include "pwm_effort_channel.h"

class WalrusBaseRobot : public hardware_interface::RobotHW
{
public:
  WalrusBaseRobot() 
 { 
   front_left_joint = PWMPositionEffortChannelPtr(new PWMPositionEffortChannel("front_left_joint", jsi, jei));
   back_left_joint = PWMPositionEffortChannelPtr(new PWMPositionEffortChannel("back_left_joint", jsi, jei));
   front_right_joint = PWMPositionEffortChannelPtr(new PWMPositionEffortChannel("front_right_joint", jsi, jei));
   back_right_joint = PWMPositionEffortChannelPtr(new PWMPositionEffortChannel("back_right_joint", jsi, jei));

   front_left_drive = PWMVelocityEffortChannelPtr(new PWMVelocityEffortChannel("front_left_drive", jsi, jei));
   back_left_drive = PWMVelocityEffortChannelPtr(new PWMVelocityEffortChannel("back_left_drive", jsi, jei));
   front_right_drive = PWMVelocityEffortChannelPtr(new PWMVelocityEffortChannel("front_right_drive", jsi, jei));
   back_right_drive = PWMVelocityEffortChannelPtr(new PWMVelocityEffortChannel("back_right_drive", jsi, jei));

   registerInterface(&jsi);
   registerInterface(&jei);
  }

  void write(){
    front_left_joint->write();
    back_left_joint->write();
    front_right_joint->write();
    back_right_joint->write();

    front_left_drive->write();
    back_left_drive->write();
    front_right_drive->write();
    back_right_drive->write();
  }
  void read(){
    front_left_joint->read();
    back_left_joint->read();
    front_right_joint->read();
    back_right_joint->read();

    front_left_drive->read();
    back_left_drive->read();
    front_right_drive->read();
    back_right_drive->read();
  }

private:
  PWMPositionEffortChannelPtr front_left_joint;
  PWMPositionEffortChannelPtr back_left_joint;
  PWMPositionEffortChannelPtr front_right_joint;
  PWMPositionEffortChannelPtr back_right_joint;

  PWMVelocityEffortChannelPtr front_left_drive;
  PWMVelocityEffortChannelPtr back_left_drive;
  PWMVelocityEffortChannelPtr front_right_drive;
  PWMVelocityEffortChannelPtr back_right_drive;

  hardware_interface::JointStateInterface jsi;
  hardware_interface::EffortJointInterface jei;
};
