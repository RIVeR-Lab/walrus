
#ifndef MAINBOARDDRIVER_H
#define MAINBOARDDRIVER_H

#include <ros/ros.h>
#include <math.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <walrus_firmware_msgs/MainBoardHighSpeedControl.h>
#include <walrus_firmware_msgs/MainBoardHighSpeedFeedback.h>
#include <walrus_firmware_msgs/MainBoardLowSpeedData.h>
#include <walrus_firmware_msgs/MainBoardControl.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <string.h>
#include <boost/lexical_cast.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <sstream>
#include <iostream>
#include <iomanip>


namespace walrus_mainboard_driver
{

    struct CellData {
        double lower_voltage, upper_voltage;
        double lower_current, upper_current;
        double lower_avg_current, upper_avg_current;
        double lower_temp, upper_temp;
        double lower_charge, upper_charge;
        double combined_voltage, combined_current, combined_avg_current, combined_charge;
        std::string mfr;
        std::string dev_name;
        std::string chemistry;
        bool present;
        bool shutdown;
    };
    
    class MainBoardDriver
    {
        public:
            MainBoardDriver(hardware_interface::ActuatorStateInterface& asi,
                  hardware_interface::EffortActuatorInterface& aei,
                  ros::NodeHandle& nh, ros::NodeHandle& pnh);
            bool init();
            void read(ros::Duration dt);
            void write(ros::Duration dt);
            void update_diagnostics();
            
        private:
            ros::NodeHandle nh, pnh;
            
            diagnostic_updater::Updater diagnostic_updater;
        
            hardware_interface::ActuatorStateInterface asi_;
            hardware_interface::EffortActuatorInterface aei_;
            
            void led_callback(const std_msgs::Float64::ConstPtr& msg, int index);
            void set_enable_callback(const std_msgs::Bool& msg);
            void hs_feedback_callback(const walrus_firmware_msgs::MainBoardHighSpeedFeedback& msg);
            void ls_data_callback(const walrus_firmware_msgs::MainBoardLowSpeedData& msg);
            void from_board_callback(const walrus_firmware_msgs::MainBoardControl& msg);
            
            static std::string formatDouble(double value, int precision);
            
            void tp_temp_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
            void int_temp_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
            void humidity_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
            void pressure_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
            void water_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat, int index);
            void pod_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat, int index);
            void mainboard_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);   
            void batt_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat, int index);
            void power_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
            void left_drive_temp_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
            void right_drive_temp_diagnsotic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
            
            void heartbeat_callback(const ros::TimerEvent&);
            
            ros::Timer heartbeat_timer;
            
            ros::Subscriber hs_feedback, ls_data, from_board, back_led, front_led, bottom_led, set_enable;
            ros::Publisher hs_control, to_board;
            
            walrus_firmware_msgs::MainBoardHighSpeedFeedback hs_feedback_msg;
            
            boost::mutex sensor_data_mutex, control_data_mutex; 
            
            //Sensor data
            double temp_sensors[10];
            double ambient_temp;
            double humidity;
            double pressure;
            bool water[6];
            CellData batteries[4];
            double main_voltage;
            double backup_voltage;
            double main_charge;
            double main_current;
            double main_avg_current;
            
            //Data for pod control and state
            bool pod_force_disable[4];
            double pod_velocity[4], pod_position[4], pod_current[4], pod_effort[4], pod_effort_cmd[4], pod_power[4];
            
            //Main Board Control
            const ros::Duration ls_data_timeout;
            const ros::Duration hs_feedback_timeout;        
            std::string main_board_status;
            uint8_t main_board_status_level;
            ros::Time last_hs_feedback;
            ros::Time last_ls_data;
            bool main_board_connected;
            bool host_enabled, board_enabled;      
            const int led_scale;      
            
            //Constants obtained from parameters
            
            //Map devices to board plugs
            int FL, FR, BR, BL;
            int MOTOR_TEMP[4];
            int CONTROLLER_TEMP[4];
            int VICOR_TEMP, TOP_PLATE_TEMP;
            int RIGHT_DRIVE_TEMP, LEFT_DRIVE_TEMP;
            int FRONT_CAM_LED, BOTTOM_CAM_LED, BACK_CAM_LED;
            int BACKUP_BATT_VOLTAGE;
            
            //Pod constants
            bool POD_AUTO_ENABLE;
            double POD_POSITION_NEUTRAL[4];
            bool POD_REV[4];
            double OUTPUT_TORQUE_PER_AMP; 
            int OUTPUT_POWER_NEUTRAL; 
            int OUTPUT_POWER_LIMIT; 
            
            //Water sensor location names
            std::string WATER_SENSE_POSITION[6];        
            
            //Levels to trigger temperature or current warnings
            //For chassis
            double TOP_PLATE_TEMP_HIGH_ABOVE;
            double TOP_PLATE_TEMP_CRITICAL_ABOVE;
            double AMBIENT_TEMP_HIGH_ABOVE;
            double AMBIENT_TEMP_CRITICAL_ABOVE;
            double PRESSURE_LOW_BELOW;
            double PRESSURE_CRITICAL_BELOW;
            double HUMIDITY_HIGH_ABOVE;
            double HUMIDITY_CRITICAL_ABOVE;
            //For pods
            double POD_MOTOR_TEMP_HIGH_ABOVE;
            double POD_MOTOR_TEMP_CRITICAL_ABOVE;
            double POD_CONTROLLER_TEMP_HIGH_ABOVE;
            double POD_CONTROLLER_TEMP_CRITICAL_ABOVE;
            double POD_MOTOR_CURRENT_HIGH_ABOVE;
            double POD_MOTOR_CURRENT_CRITICAL_ABOVE;
            //For main board
            int HS_FEEDBACK_TIMEOUT;
            //For batteries
            double MAIN_BATT_VOLTAGE_LOW_BELOW;
            double MAIN_BATT_CHARGE_LOW_BELOW;
            double MAIN_BATT_TEMP_HIGH_ABOVE;
            double BACKUP_BATT_VOLTS_PER_COUNT;
            //For power systems
            double VICOR_TEMP_HIGH_ABOVE;
            double VICOR_TEMP_CRITICAL_ABOVE;
            double BACKUP_BATT_VOLTAGE_LOW_BELOW;
            double MAIN_VOLTAGE_LOW_BELOW;
            double MAIN_CHARGE_LOW_BELOW;
            //For drive temps
            double DRIVE_MOTOR_TEMP_HIGH_ABOVE;
            double DRIVE_MOTOR_TEMP_CRITICAL_ABOVE;
            
    };
    
}

#endif
