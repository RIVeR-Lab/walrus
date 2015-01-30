#include "walrus_mainboard_driver/walrus_mainboard_driver.h"

namespace walrus_mainboard_driver
{
	
	MainBoardDriver::MainBoardDriver(hardware_interface::ActuatorStateInterface& asi,
				  hardware_interface::EffortActuatorInterface &aei,
				  ros::NodeHandle& nh, ros::NodeHandle& pnh)
	: asi_(&asi), aei_(&aei), diagnostic_updater(nh, pnh)
	{
		tx = nh.advertise<walrus_firmware_msgs::MainBoardTXMsg>("/walrus/main_board/tx", 1000);
		rx = nh.subscribe("/walrus/main_board/rx", boost::bind(&MainBoardDriver::rx_callback, this, _1));
		
		hardware_interface::ActuatorStateHandle state_handle0("walrus/front_left_pod_joint_actuator", &rx_msg.pod_position[0], NULL, &rx_msg.pod_current[0]);
		asi.registerHandle(state_handl0);
		hardware_interface::ActuatorStateHandle state_handle1("walrus/front_right_pod_joint_actuator", &rx_msg.pod_position[1], NULL, &rx_msg.pod_current[1]);
		asi.registerHandle(state_handle1);
		hardware_interface::ActuatorStateHandle state_handle2("walrus/back_right_pod_joint_actuator", &rx_msg.pod_position[2], NULL, &rx_msg.pod_current[2]);
		asi.registerHandle(state_handle2);
		hardware_interface::ActuatorStateHandle state_handle3("walrus/back_left_pod_joint_actuator", &rx_msg.pod_position[3], NULL, &rx_msg.pod_current[3]);
		asi.registerHandle(state_handle3);
		
		hardware_interface ActuatorHandle effort_handle0(state_handle0, &tx_msg.motor_power[0]);
		aei.registerHandle(effort_handle0);
		hardware_interface ActuatorHandle effort_handle0(state_handle1, &tx_msg.motor_power[1]);
		aei.registerHandle(effort_handle1);
		hardware_interface ActuatorHandle effort_handle0(state_handle2, &tx_msg.motor_power[2]);
		aei.registerHandle(effort_handle2);
		hardware_interface ActuatorHandle effort_handle0(state_handle3, &tx_msg.motor_power[3]);
		aei.registerHandle(effort_handle3);
	}
	
	bool MainBoardDriver::init()
	{
		return true;
	}
	
	void MainBoardDriver::read()
	{
	}
	
	void MainBoardDriver::write()
	{		
		tx.publish(tx_msg);
	}
	
	void MainBoardDriver::rx_callback(const walrus_firmware_msgs::MainBoardRXMsg& msg)
	{
		rx_msgs = msg;
	}
	
	void update_diagnostics()
	{
		diagnostic_updater.update();
	}
}