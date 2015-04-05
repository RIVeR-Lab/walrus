#ifndef WALRUS_ROBOT_BASE_H_
#define WALRUS_ROBOT_BASE_H_

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

class WalrusRobotBase : public RobotHW
{
 public:
  WalrusRobotBase(ros::NodeHandle nh, ros::NodeHandle pnh);

  bool loadTransmissions(const std::vector<std::string>& actuator_names);

  virtual void write(ros::Duration dt) = 0;
  virtual void read(ros::Duration dt) = 0;
  virtual void update_diagnostics() = 0;

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  RobotTransmissions robot_transmissions_;

private:
  boost::scoped_ptr<TransmissionInterfaceLoader> transmission_loader_;
};

}

#endif
