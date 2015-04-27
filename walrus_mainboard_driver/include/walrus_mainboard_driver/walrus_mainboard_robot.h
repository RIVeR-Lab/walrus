
#ifndef MAINBOARDDRIVER_H
#define MAINBOARDDRIVER_H

#include <ros/ros.h>
#include <math.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <walrus_firmware_msgs/MainBoardSensorData.h>
#include <walrus_firmware_msgs/MainBoardControl.h>
#include <std_msgs/Float64.h>
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
    
  class MainBoardRobot
    {
        public:
            MainBoardRobot(ros::NodeHandle& nh, ros::NodeHandle& pnh);
            
            void update_diagnostics();
            
        private:
            ros::NodeHandle nh_, pnh_;
        
            diagnostic_updater::Updater diagnostic_updater;
            
            void led_callback(const std_msgs::Float64::ConstPtr& msg, int index);
            void sensor_data_callback(const walrus_firmware_msgs::MainBoardSensorData& msg);
            void from_board_callback(const walrus_firmware_msgs::MainBoardControl& msg);
            
            static std::string formatDouble(double value, int precision);
            
            void tp_temp_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
            void int_temp_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
            void humidity_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
            void pressure_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
            void water_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat, int index);
            void pod_temp_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat, int index);
            void mainboard_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);   
            void batt_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat, int index);
            void power_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
            void left_drive_temp_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
            void right_drive_temp_diagnsotic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
            
            void heartbeat_callback(const ros::TimerEvent&);
            
            ros::Timer heartbeat_timer;
            
            ros::Subscriber sensor_data, from_board, back_led, front_led, bottom_led;
            ros::Publisher to_board;
            
            boost::mutex sensor_data_mutex;
            
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
            
            //Main Board Control
            const ros::Duration sensor_data_timeout;     
            ros::Time last_sensor_data;
            bool main_board_connected;    
            const int led_scale;      
            
            //Constants obtained from parameters
           
            //Map devices to board plugs
            int FL_POD_MOTOR_TEMP, FR_POD_MOTOR_TEMP, BL_POD_MOTOR_TEMP, BR_POD_MOTOR_TEMP;
            int VICOR_TEMP, TOP_PLATE_TEMP;
            int RIGHT_DRIVE_TEMP, LEFT_DRIVE_TEMP;
            int FRONT_CAM_LED, BOTTOM_CAM_LED, BACK_CAM_LED;
            int BACKUP_BATT_VOLTAGE;
            
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
