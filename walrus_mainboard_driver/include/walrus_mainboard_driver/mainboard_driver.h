
#ifndef MAINBOARDDRIVER_H
#define MAINBOARDDRIVER_H

#include <ros/ros.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <walrus_firmware_msgs/MainBoardTXMsg.h>SS
#include <walrus_firmware_msgs/MainBoardRXMsg.h>

namespace walrus_mainboard_driver
{
	
	class MainBoardDriver : public hardware_interface::RobotHW 
	{
		public:
			MainBoardDriver(hardware_interface::ActuatorStateInterface& asi,
				  hardware_interface::EffortActuatorInterface& aei,
				  ros::NodeHandle& nh, ros::NodeHandle& pnh);
			bool init();
			void read();
			void write();
			void update_diagnostics();
			
		private:
			diagnostic_updater::Updater diagnostic_updater;
		
			hardware_interface::AcuatorStateInterface asi_;
			hardware_interface::PositionActuatorInterface api_;
			hardware_interface::VelocityActuatorInterface avi_;
			
			void rx_callback(const walrus_firmware_msgs::MainBoardRXMsg& msg);
			
			ros::Subscriber rx;
			ros::Publisher tx;
			
			walrus_firmware_msgs::MainBoardTXMsg tx_msg;
			walrus_firmware_msgs::MainBoardRXMsg rx_msg;
	}
	
	
}

#endif