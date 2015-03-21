#include "walrus_boomboard_driver/walrus_boomboard_robot.h"
#include <boost/assign/list_of.hpp>

namespace walrus_boomboard_driver
{

    BoomBoardRobot::BoomBoardRobot(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : WalrusRobotBase(nh, pnh), diagnostic_updater(nh, pnh)
    {
        tx = nh.advertise<walrus_firmware_msgs::BoomBoardTXMsg>("boom_board/tx", 1000);
        rx = nh.subscribe("boom_board/rx", 1000, &BoomBoardRobot::rx_callback, this);

        hardware_interface::ActuatorStateHandle state_handle_deploy("walrus/boom/deploy_joint_actuator", &deploy_position, &deploy_velocity, &deploy_effort);
        asi_.registerHandle(state_handle_deploy);
        hardware_interface::ActuatorStateHandle state_handle_pan("walrus/boom/pan_joint_actuator", &pan_position, &pan_velocity, &pan_effort);
        asi_.registerHandle(state_handle_pan);
        hardware_interface::ActuatorStateHandle state_handle_tilt("walrus/boom/tilt_joint_actuator", &tilt_position, &tilt_velocity, &tilt_effort);
        asi_.registerHandle(state_handle_tilt);

        hardware_interface::ActuatorHandle effort_handle_deploy(state_handle_deploy, &deploy_effort_cmd);
        aei_.registerHandle(effort_handle_deploy);
        hardware_interface::ActuatorHandle effort_handle_pan(state_handle_pan, &pan_effort_cmd);
        aei_.registerHandle(effort_handle_pan);
        hardware_interface::ActuatorHandle effort_handle_tilt(state_handle_tilt, &tilt_effort_cmd);
        aei_.registerHandle(effort_handle_tilt);

        diagnostic_updater.setHardwareID("Walrus Boom Board");
    }

    bool BoomBoardRobot::init()
    {
      registerInterface(&asi_);
      registerInterface(&aei_);

      std::vector<std::string> actuator_names = boost::assign::list_of
	("walrus/boom/deploy_joint_actuator")
	("walrus/boom/pan_joint_actuator")
	("walrus/boom/tilt_joint_actuator");
      return loadTransmissions(actuator_names);
    }

    void BoomBoardRobot::read(ros::Duration dt)
    {
      deploy_velocity = 0;
      deploy_position = rx_msg.deploy_position;
      deploy_effort = 0;
      pan_velocity = 0;
      pan_position = rx_msg.pan_position;
      pan_effort = rx_msg.pan_current;
      tilt_velocity = 0;
      tilt_position = rx_msg.tilt_position;
      tilt_effort = rx_msg.tilt_current;
      robot_transmissions_.get<transmission_interface::ActuatorToJointStateInterface>()->propagate();
    }

    void BoomBoardRobot::write(ros::Duration dt)
    {
      robot_transmissions_.get<transmission_interface::JointToActuatorEffortInterface>()->propagate();
      tx_msg.deploy_power = deploy_effort_cmd;
      tx_msg.pan_power = pan_effort_cmd;
      tx_msg.tilt_power = tilt_effort_cmd;
      tx.publish(tx_msg);
    }

    void BoomBoardRobot::rx_callback(const walrus_firmware_msgs::BoomBoardRXMsg& msg)
    {
        rx_msg = msg;
    }

    void BoomBoardRobot::update_diagnostics()
    {
        diagnostic_updater.update();
    }
}
