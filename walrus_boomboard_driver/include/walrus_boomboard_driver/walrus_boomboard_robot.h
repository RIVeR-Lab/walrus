
#ifndef BOOMBOARDDRIVER_H
#define BOOMBOARDDRIVER_H

#include <ros/ros.h>
#include <math.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <walrus_firmware_msgs/BoomBoardHighSpeedControl.h>
#include <walrus_firmware_msgs/BoomBoardHighSpeedFeedback.h>
#include <walrus_firmware_msgs/BoomBoardLowSpeedData.h>
#include <walrus_firmware_msgs/BoomBoardControl.h>
#include <walrus_base_hw/walrus_robot_base.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <string.h>
#include <boost/lexical_cast.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <sstream>
#include <iostream>
#include <iomanip>


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

            void led_callback(const std_msgs::Float64::ConstPtr& msg);
            void set_enable_callback(const std_msgs::Bool& msg);
            void hs_feedback_callback(const walrus_firmware_msgs::BoomBoardHighSpeedFeedback& msg);
            void ls_data_callback(const walrus_firmware_msgs::BoomBoardLowSpeedData& msg);
            void from_board_callback(const walrus_firmware_msgs::BoomBoardControl& msg);
            
            static std::string formatDouble(double value, int precision);
            
            void CO_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
            void CNG_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
            void LPG_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
            void H_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
            void temp_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
            void humidity_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
            void tilt_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
            void pan_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
            void deploy_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
            void boomboard_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
 
            
            void heartbeat_callback(const ros::TimerEvent&);

            ros::Timer heartbeat_timer;
            
            ros::Subscriber hs_feedback, ls_data, from_board, cam_led, set_enable;
            ros::Publisher hs_control, to_board;

            walrus_firmware_msgs::BoomBoardHighSpeedFeedback hs_feedback_msg;
            
            boost::mutex sensor_data_mutex, control_data_mutex;

            //Sensor data
            double temp;
            double humidity;
            double CO;
            double CNG;
            double LPG;
            double H;
            
            //Motor control and stat data
            double deploy_effort_cmd;
            double deploy_velocity;
            double deploy_position_raw;
            double deploy_position;
            double deploy_effort;
            double deploy_power;
            bool deploy_limit_max;
            bool deploy_limit_min;
            double pan_effort_cmd;
            double pan_velocity;
            double pan_position_raw;
            double pan_position;
            double pan_current;
            double pan_effort;
            double pan_power;
            bool pan_limit_max;
            bool pan_limit_min;
            double tilt_effort_cmd;
            double tilt_velocity;
            double tilt_position;
            double tilt_position_raw;
            double tilt_current;
            double tilt_effort;
            double tilt_power;
            bool tilt_limit_max;
            bool tilt_limit_min;
            
            //Boom Board Control
            const ros::Duration ls_data_timeout;
            const ros::Duration hs_feedback_timeout;        
            std::string boom_board_status;
            uint8_t boom_board_status_level;
            ros::Time last_hs_feedback;
            ros::Time last_ls_data;
            bool boom_board_connected;
            bool host_enabled, board_enabled;      
            const int led_scale; 
            
            //Constants obtained from parameters     
            int CO_HIGH_LEVEL;
            int CNG_HIGH_LEVEL;
            int LPG_HIGH_LEVEL;
            int H_HIGH_LEVEL;
            int PAN_POSITION_MAX;
            int PAN_POSITION_MIN;
            int PAN_POSITION_NEUTRAL;
            double PAN_POSITION_SCALE;
            bool PAN_MOTOR_REV;
            bool PAN_POT_REV;
            double PAN_TORQUE_PER_AMP;
            int TILT_POSITION_MAX;
            int TILT_POSITION_MIN;
            int TILT_POSITION_NEUTRAL;
            double TILT_POSITION_SCALE;
            bool TILT_MOTOR_REV;
            bool TILT_POT_REV;
            double TILT_TORQUE_PER_AMP;
            int DEPLOY_POSITION_MAX;
            int DEPLOY_POSITION_MIN;
            int DEPLOY_POSITION_NEUTRAL;
            double DEPLOY_POSITION_SCALE;
            bool DEPLOY_MOTOR_REV;
            bool DEPLOY_POT_REV;
            double MOTOR_OUTPUT_SCALE;
            bool AUTO_ENABLE;
            
    };
}

#endif
