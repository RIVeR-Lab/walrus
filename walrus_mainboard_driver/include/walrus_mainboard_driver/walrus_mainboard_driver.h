
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
#include <string.h>

namespace walrus_mainboard_driver
{

    struct CellData {
        double lower_voltage, upper_voltage;
        double lower_current, upper_current;
        double lower_avg_current, upper_avg_current;
        double lower_temp, upper_temp;
        double lower_charge, upper_charge;
        double combined_voltage, combined_current, combined_avg_current, combined_charge;
        string mfr;
        string dev_name;
        string chemistry;
        bool present;
        bool shutdown; 
        
    }
	
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
			
		private:
		    ros::NodeHandle nh, phn;
		    
			diagnostic_updater::Updater diagnostic_updater;
		
			hardware_interface::ActuatorStateInterface asi_;
			hardware_interface::EffortActuatorInterface aei_;
			
			void hs_feedback_callback(const walrus_firmware_msgs::MainBoardHighSpeedFeedback& msg);
			void ls_data_callback(const walrus_firmware_msgs::MainBoardLowSpeedDatak& msg);
			void from_board_callback(const walrus_firmware_msgs::MainBoardControl& msg);
			
			void chassis_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
			void FLPod_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
			void FRPod_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
			void BRPod_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
			void BLPod_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
			void mainboard_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);   
			void batt_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat, int index);
			void power_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
			void left_drive_temp_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat);
			void right_drive_temp_diagnsotic_callback((diagnostic_updater::DiagnosticStatusWrapper &stat);
			
			void heartbeat_callback(const ros::TimerEvent&);
			
			ros::Subscriber hs_feedback, ls_data, from_board;
			ros::Publisher hs_control, to_board;
			
		    walrus_firmware_msgs::MainBoardHighSpeedFeedback hs_feedback;
		    
			std:mutex sensor_data_mutex, control_data_mutex; 
			
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
			string main_board_status;
			uint8_t main_board_status_level;
			
			//Data for pod control and state
			ros::Time last_hs_msg;
			bool main_board_connected;
			bool host_enabled, board_enabled;
			bool FLPod_force_disable, FRPod_force_disable, BRPod_force_disable, BLPod_force_disable;
			double FLPod_velocity, FLPod_position, FLPod_current, FLPod_effort, FLPod_effort_cmd, FLPod_power;
			double FRPod_velocity, FRPod_position, FRPod_current, FRPod_effort, FRPod_effort_cmd, FRPod_power;
			double BRPod_velocity, BRPod_position, BRPod_current, BRPod_effort, BRPod_effort_cmd, BRPod_power;
			double BLPod_velocity, BLPod_position, BLPod_current, BLPod_effort, BLPod_effort_cmd, BLPod_power;
			
			//Map devices to board plugs
			int FL, FR, BR, BL;
			int FL_MOTOR_TEMP, FR_MOTOR_TEMP, BR_MOTOR_TEMP, BL_MOTOR_TEMP;
			int FL_CONTROLLER_TEMP, FR_CONTROLLER_TEMP, BR_CONTROLLER_TEMP, BL_CONTROLLER_TEMP;
			int VICOR_TEMP, TOP_PLATE_TEMP;
			int RIGHT_DRIVE_TEMP, LEFT_DRIVE_TEMP;
			int FRONT_CAM_LED, BOTTOM_CAM_LED, BACK_CAM_LED;
			int BACKUP_BATT_VOLTAGE;
			
			//Levels to trigger temperature or current warnings
		    //For chassis
		    double TOP_PLATE_TEMP_WARN_ABOVE;
		    double TOP_PLATE_TEMP_ERROR_ABOVE;
		    double AMBIENT_TEMP_WARN_ABOVE;
		    double AMBIENT_TEMP_ERROR_ABOVE;
		    double PRESSURE_WARN_BELOW;
		    double PRESSURE_ERROR_BELOW;
		    double HUMIDITY_WARN_ABOVE;
		    double HUMIDITY_ERROR_ABOVE;
		    string WATER_SENSE_POSITION[6];
		    //For pods
		    double POD_MOTOR_TEMP_WARN_ABOVE;
		    double POD_MOTOR_TEMP_ERROR_ABOVE;
		    double POD_CONTROLLER_TEMP_WARN_ABOVE;
		    double POD_CONTROLLER_TEMP_ERROR_ABOVE;
		    double POD_MOTOR_CURRENT_WARN_ABOVE;
		    double POD_MOTOR_CURRENT_ERROR_ABOVE;
		    //For main board
		    double HS_FEEDBACK_TIMEOUT;
		    //For batteries
		    double MAIN_BATT_VOLTAGE_WARN_BELOW;
		    double MAIN_BATT_CURRENT_WARN_ABOVE;
		    double MAIN_BATT_CHARGE_WARN_BELOW;
		    double MAIN_BATT_TEMP_WARN_ABOVE;
		    //For power systems
		    double VICOR_TEMP_WARN_ABOVE;
		    double VICOR_TEMP_ERROR_ABOVE;
		    double BACKUP_BATT_VOLTAGE_WARN_BELOW;
		    double MAIN_VOLTAGE_WARN_BELOW;
		    double MAIN_CHARGE_WARN_BELOW;
		    //For drive temps
		    double DRIVE_MOTOR_TEMP_WARN_ABOVE;
		    double DRIVE_MOTOR_TEMP_ERROR_ABOVE;
		    
	};
	
}

#endif
