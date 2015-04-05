
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
#include <walrus_base_hw/walrus_robot_base.h>

namespace walrus_boomboard_driver
{
  class BoomBoardRobot : public walrus_base_hw::WalrusRobotBase
    {
        public:
            BoomBoardRobot(ros::NodeHandle& nh, ros::NodeHandle& pnh);
            bool init();
            void read(ros::Duration dt);
            void write(ros::Duration dt);
            void update_diagnostics();

        private:
            diagnostic_updater::Updater diagnostic_updater;

            hardware_interface::ActuatorStateInterface asi_;
            hardware_interface::EffortActuatorInterface aei_;

            void rx_callback(const walrus_firmware_msgs::BoomBoardRXMsg& msg);

            ros::Subscriber rx;
            ros::Publisher tx;

            walrus_firmware_msgs::BoomBoardTXMsg tx_msg;
            walrus_firmware_msgs::BoomBoardRXMsg rx_msg;

            double deploy_effort_cmd;
            double deploy_velocity;
            double deploy_position;
            double deploy_effort;
            double pan_effort_cmd;
            double pan_velocity;
            double pan_position;
            double pan_effort;
            double tilt_effort_cmd;
            double tilt_velocity;
            double tilt_position;
            double tilt_effort;
    };
}

#endif
