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

namespace walrus_base_hw {

using namespace transmission_interface;
using namespace hardware_interface;

class WalrusBaseRobot : public RobotHW
{
 public:
  WalrusBaseRobot(ros::NodeHandle nh, ros::NodeHandle pnh);

  bool init();

  void write();
  void read();
  void update_diagnostics();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  RobotTransmissions robot_transmissions_;
  boost::scoped_ptr<TransmissionInterfaceLoader> transmission_loader_;
  boost::shared_ptr<epos_hardware::EposManager> epos_manager_;

  ActuatorStateInterface as_interface_;
  VelocityActuatorInterface av_interface_;
  PositionActuatorInterface ap_interface_;

  // Temporary fake actuators
  typedef struct FakeActuatorData {
    FakeActuatorData(const std::string& name)
      : name(name), position(0), velocity(0), effort(0), cmd(0) {}
    std::string name;
    double position;
    double velocity;
    double effort;
    double cmd;
  } FakeActuatorData;
  std::vector<boost::shared_ptr<FakeActuatorData> > fake_actuator_data;
  void createFakeActuator(const std::string& name);

};

}

#endif
