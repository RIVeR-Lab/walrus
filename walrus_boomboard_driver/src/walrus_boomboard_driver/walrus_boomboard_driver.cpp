
#ifndef BOOMBOARDDRIVER_H
#define BOOMBOARDDRIVER_H

#include <ros/ros.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <walrus_firmware_msgs/BoomBoardTXMsg.h>
#include <walrus_firmware_msgs/BoomBoardRXMsg.h>

namespace walrus_boomboard_driver
{
	class BoobBoardDriver
	{
		public:
			BoomBoardDriver(hardware_interface::ActuatorStateInterface& asi,
				  hardware_interface::EffortActuatorInterface& aei,
				  ros::NodeHandle& nh, ros::NodeHandle& pnh);
			bool init();
			void read();
			void write();
			void update_diagnostics();
			
		private:
			diagnostic_updater::Updater diagnostic_updater;
		
			hardware_interface::AcuatorStateInterface asi_;
			hhardware_interface::EffortActuatorInterface aei_;
			
			void rx_callback(const walrus_firmware_msgs::BoomBoardRXMsg& msg);
			
			ros::Subscriber rx;
			ros::Publisher tx;
			
			walrus_firmware_msgs::BoomBoardTXMsg tx_msg;
			walrus_firmware_msgs::BoomBoardRXMsg rx_msg;
	}
}

#endif