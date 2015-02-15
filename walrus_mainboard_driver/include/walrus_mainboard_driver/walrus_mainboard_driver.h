
#ifndef MAINBOARDDRIVER_H
#define MAINBOARDDRIVER_H

#include <ros/ros.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <walrus_firmware_msgs/MainBoardTXMsg.h>
#include <walrus_firmware_msgs/MainBoardRXMsg.h>

namespace walrus_mainboard_driver
{
	
	class MainBoardDriver
	{
		public:
			MainBoardDriver(hardware_interface::ActuatorStateInterface& asi,
				  hardware_interface::EffortActuatorInterface& aei,
				  ros::NodeHandle& nh, ros::NodeHandle& pnh);
			bool init();
			void read();
			void write();
			void update_diagnostics();
			
			static const int FL;
			static const int FR;
			static const int BR;
			static const int BL;
			
		private:
			diagnostic_updater::Updater diagnostic_updater;
		
			hardware_interface::ActuatorStateInterface asi_;
			hardware_interface::EffortActuatorInterface aei_;
			
			void hs_feedback_callback(const walrus_firmware_msgs::MainBoardHighSpeedFeedback& msg);
			void ls_data_callback(const walrus_firmware_msgs::MainBoardLowSpeedDatak& msg);
			void from_board_callback(const walrus_firmware_msgs::MainBoardControl& msg);
			
			ros::Subscriber hs_feedback, ls_data, from_board;
			ros::Publisher hs_control, to_board;
			
			walrus_firmware_msgs::MainBoardTXMsg tx_msg;
			walrus_firmware_msgs::MainBoardRXMsg rx_msg;
			
			double FLPod_velocity, FLPod_position, FLPod_effort, FLPod_effort_cmd;
			double FRPod_velocity, FRPod_position, FRPod_effort, FRPod_effort_cmd;
			double BRPod_velocity, BRPod_position, BRPod_effort, BRPod_effort_cmd;
			double BLPod_velocity, BLPod_position, BLPod_effort, BLPod_effort_cmd;
	};
	
	
}

#endif
