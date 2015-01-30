#include "walrus_boomboard_driver/walrus_boomboard_driver.h"

namespace walrus_board_driverS
{
	
	BoardDriver::BoardDriver(hardware_interface::ActuatorStateInterface& asi,
				  hardware_interface::EffortActuatorInterface &aei,
				  ros::NodeHandle& nh, ros::NodeHandle& pnh)
	: asi_(&asi), aei_(&aei), diagnostic_updater(nh, pnh)
	{
		tx = nh.advertise<walrus_firmware_msgs::BoardTXMsg>("/walrus/_board/tx", 1000);
		rx = nh.subscribe("/walrus/_board/rx", boost::bind(&BoardDriver::rx_callback, this, _1));
		
		hardware_interface::ActuatorStateHandle state_handle0("walrus/boom_deploy_joint_actuator", &rx_msg.deploy_position, NULL, NULL);
		asi.registerHandle(state_handl0);
		hardware_interface::ActuatorStateHandle state_handle1("walrus/boom_pan_joint_actuator", &rx_msg.pan_position, NULL, &rx_msg.pan_current);
		asi.registerHandle(state_handle1);
		hardware_interface::ActuatorStateHandle state_handle2("walrus/boom_title_joint_acutator", &rx_msg.tilt_position, NULL, &rx_msg.tilt_current);
		asi.registerHandle(state_handle2);
		
		hardware_interface ActuatorHandle effort_handle0(state_handle0, &tx_msg.deploy_power]);
		aei.registerHandle(effort_handle0);
		hardware_interface ActuatorHandle effort_handle0(state_handle1, &tx_msg.pan_power);
		aei.registerHandle(effort_handle1);
		hardware_interface ActuatorHandle effort_handle0(state_handle2, &tx_msg.tilt_power);
		aei.registerHandle(effort_handle2);
	}
	
	bool BoardDriver::init()
	{
		return true;
	}
	
	void BoardDriver::read()
	{
	}
	
	void BoardDriver::write()
	{		
		tx.publish(tx_msg);
	}
	
	void BoardDriver::rx_callback(const walrus_firmware_msgs::BoardRXMsg& msg)
	{
		rx_msgs = msg;
	}
	
	void update_diagnostics()
	{
		diagnostic_updater.update();
	}
}