#include "walrus_boomboard_driver/walrus_boomboard_driver.h"

namespace walrus_boomboard_driver
{
	
	BoomBoardDriver::BoomBoardDriver(hardware_interface::ActuatorStateInterface& asi,
				  hardware_interface::EffortActuatorInterface &aei,
				  ros::NodeHandle& nh, ros::NodeHandle& pnh)
	: asi_(asi), aei_(aei), diagnostic_updater(nh, pnh)
	{
		tx = nh.advertise<walrus_firmware_msgs::BoomBoardTXMsg>("/walrus/boom_board/tx", 1000);
		rx = nh.subscribe("/walrus/boom_board/rx", 1000, &BoomBoardDriver::rx_callback, this);
		
		hardware_interface::ActuatorStateHandle state_handle0("walrus/boom/deploy_joint_actuator", &deploy_position, &deploy_velocity, &deploy_effort);
		asi.registerHandle(state_handle0);
		hardware_interface::ActuatorStateHandle state_handle1("walrus/boom/pan_joint_actuator", &pan_position, &pan_velocity, &pan_effort);
		asi.registerHandle(state_handle1);
		hardware_interface::ActuatorStateHandle state_handle2("walrus/boom/tilt_joint_actuator", &tilt_position, &tilt_velocity, &tilt_effort);
		asi.registerHandle(state_handle2);
		
		hardware_interface::ActuatorHandle effort_handle0(state_handle0, &deploy_effort_cmd);
		aei.registerHandle(effort_handle0);
		hardware_interface::ActuatorHandle effort_handle1(state_handle1, &pan_effort_cmd);
		aei.registerHandle(effort_handle1);
		hardware_interface::ActuatorHandle effort_handle2(state_handle2, &tilt_effort_cmd);
		aei.registerHandle(effort_handle2);
	}
	
	bool BoomBoardDriver::init()
	{
		return true;
	}
	
	void BoomBoardDriver::read()
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
	}
	
	void BoomBoardDriver::write()
	{		
		tx_msg.deploy_power = deploy_effort_cmd;
		tx_msg.pan_power = pan_effort_cmd;
		tx_msg.tilt_power = tilt_effort_cmd;
		tx.publish(tx_msg);
	}
	
	void BoomBoardDriver::rx_callback(const walrus_firmware_msgs::BoomBoardRXMsg& msg)
	{
		rx_msg = msg;
	}
	
	void BoomBoardDriver::update_diagnostics()
	{
		diagnostic_updater.update();
	}
}
