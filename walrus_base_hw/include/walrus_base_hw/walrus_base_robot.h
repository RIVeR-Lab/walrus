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
#include <epos_hardware/epos_manager.h>
#include <walrus_mainboard_driver/walrus_mainboard_driver.h>
#include <walrus_boomboard_driver/walrus_boomboard_driver.h>

namespace walrus_base_hw {

using namespace transmission_interface;
using namespace hardware_interface;

class WalrusBaseRobot : public RobotHW
{
 public:
  WalrusBaseRobot(ros::NodeHandle nh, ros::NodeHandle pnh);

  bool init();

  void write(ros::Duration dt);
  void read(ros::Duration dt);
  void update_diagnostics();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  
  ActuatorStateInterface as_interface_;
  VelocityActuatorInterface av_interface_;
  PositionActuatorInterface ap_interface_;
  EffortActuatorInterface ae_interface_;

  RobotTransmissions robot_transmissions_;
  boost::scoped_ptr<TransmissionInterfaceLoader> transmission_loader_;
  boost::shared_ptr<epos_hardware::EposManager> epos_manager_;
  walrus_mainboard_driver::MainBoardDriver mainboard_;
  walrus_boomboard_driver::BoomBoardDriver boomboard_;

};

}

#endif
