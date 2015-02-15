#include "walrus_mainboard_driver/walrus_mainboard_driver.h"

namespace walrus_mainboard_driver
{

	const int MainBoardDriver::FL = 0;
	const int MainBoardDriver::FR = 1;
	const int MainBoardDriver::BR = 2;
	const int MainBoardDriver::BL = 3;
	
	MainBoardDriver::MainBoardDriver(hardware_interface::ActuatorStateInterface& asi,
				  hardware_interface::EffortActuatorInterface &aei,
				  ros::NodeHandle& nh, ros::NodeHandle& pnh)
	: asi_(asi), aei_(aei), diagnostic_updater(nh, pnh)
	{
		hs_control = nh.advertise<walrus_firmware_msgs::MainBoardHighSpeedControl>("/walrus/main_board/hs_control", 1000);
		to_board = nh.advertise<walrus_firmware_msgs::MainBoardControl>("/walrus/main_board/PC_to_board_control", 1000);
		hs_feedback = nh.subscribe("/walrus/main_board/hs_feedback", 1000, &MainBoardDriver::hs_feedback_callback, this);
		ls_control = nh.subscribe("/walrus/main_board/ls_data", 1000, &MainBoardDriver::ls_data_callback, this);
		from_board = nh.subscribe("/walrus/main_board/board_to_PC_control", 1000, &MainBoardDriver::from_board_callback, this);
		
		hardware_interface::ActuatorStateHandle state_handle0("walrus/front_left_pod_joint_actuator", &FLPod_position, &FLPod_velocity, &FLPod_effort);
		asi.registerHandle(state_handle0);
		hardware_interface::ActuatorStateHandle state_handle1("walrus/front_right_pod_joint_actuator", &FRPod_position, &FRPod_velocity, &FRPod_effort);
		asi.registerHandle(state_handle1);
		hardware_interface::ActuatorStateHandle state_handle2("walrus/back_right_pod_joint_actuator", &BRPod_position, &BRPod_velocity, &BRPod_effort);
		asi.registerHandle(state_handle2);
		hardware_interface::ActuatorStateHandle state_handle3("walrus/back_left_pod_joint_actuator", &BLPod_position, &BLPod_velocity, &BLPod_effort);
		asi.registerHandle(state_handle3);
		
		hardware_interface::ActuatorHandle effort_handle0(state_handle0, &FLPod_effort_cmd);
		aei.registerHandle(effort_handle0);
		hardware_interface::ActuatorHandle effort_handle1(state_handle1, &FRPod_effort_cmd);
		aei.registerHandle(effort_handle1);
		hardware_interface::ActuatorHandle effort_handle2(state_handle2, &BRPod_effort_cmd);
		aei.registerHandle(effort_handle2);
		hardware_interface::ActuatorHandle effort_handle3(state_handle3, &BLPod_effort_cmd);
		aei.registerHandle(effort_handle3);
	}
	
	bool MainBoardDriver::init()
	{
		return true;
	}
	
	void MainBoardDriver::read()
	{
		FLPod_velocity = 0;
		FLPod_position = rx_msg.pod_position[FL];
		FLPod_effort = rx_msg.motor_current[FL];
		FRPod_velocity = 0;
		FRPod_position = rx_msg.pod_position[FR];
		FRPod_effort = rx_msg.motor_current[FR];
		BRPod_velocity = 0;
		BRPod_position = rx_msg.pod_position[BR];
		BRPod_effort = rx_msg.motor_current[BR];
		BLPod_velocity = 0;
		BLPod_position = rx_msg.pod_position[BL];
		BLPod_effort = rx_msg.motor_current[BL];
	}
	
	void MainBoardDriver::write()
	{		
		tx_msg.motor_power[FL] = FLPod_effort_cmd;
		tx_msg.motor_power[FR] = FRPod_effort_cmd;
		tx_msg.motor_power[BR] = BRPod_effort_cmd;
		tx_msg.motor_power[BL] = BLPod_effort_cmd;
		tx.publish(tx_msg);
	}
	
	void MainBoardDriver::update_diagnostics()
	{
		diagnostic_updater.update();
	}
	
	void MainBoardDriver::hs_feedback_callback(const walrus_firmware_msgs::MainBoardHighSpeedFeedback& msg)
	{
	}
	
	void MainBoardDriver::ls_data_callback(const walrus_firmware_msgs::MainBoardLowSpeedDatak& msg)
	{
    }
    
	void MainBoardDriver::from_board_callback(const walrus_firmware_msgs::MainBoardControl& msg)
	{
    }

}
