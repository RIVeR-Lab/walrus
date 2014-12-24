#ifndef WALRUS_BASE_ROBOT_H_
#define WALRUS_BASE_ROBOT_H_

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_info.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <transmission_interface/transmission_parser.h>
#include <boost/scoped_ptr.hpp>


namespace walrus_base_hw {

using namespace transmission_interface;
using namespace hardware_interface;

class WalrusBaseRobot : public RobotHW
{
 public:
  WalrusBaseRobot(ros::NodeHandle nh = ros::NodeHandle(), std::string robot_ns="walrus/");

  bool init();

  void write();
  void read();

 private:
  ros::NodeHandle nh_;
  std::string robot_ns_;
  RobotTransmissions robot_transmissions_;
  boost::scoped_ptr<TransmissionInterfaceLoader> transmission_loader_;

  ActuatorStateInterface as_interface_;
  EffortActuatorInterface ae_interface_;
  VelocityActuatorInterface av_interface_;
};

}

#endif
