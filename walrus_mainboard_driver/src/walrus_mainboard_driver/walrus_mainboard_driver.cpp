#include "walrus_mainboard_driver/walrus_mainboard_driver.h"

#define PI 3.14159265358979323846

using namespace std;

namespace walrus_mainboard_driver
{
        
	MainBoardDriver::MainBoardDriver(hardware_interface::ActuatorStateInterface& asi,
				  hardware_interface::EffortActuatorInterface &aei,
				  ros::NodeHandle& nh, ros::NodeHandle& pnh)
	: asi_(asi), aei_(aei), diagnostic_updater(nh, pnh), nh(nh), pnh(pnh)
	{	    		
	    //Load parameters	
	    pnh.param("frontleft_pod_id", FL, 1);
	    pnh.param("frontright_pod_id", FR, 2);
	    pnh.param("backright_pod_id", BR, 3);
	    pnh.param("backleft_pod_id", BL, 4);
	    pnh.param("frontleft_motor_temp_id", MOTOR_TEMP[FL-1], 1);
	    pnh.param("frontright_motor_temp_id", MOTOR_TEMP[FR-1], 2);
	    pnh.param("backright_motor_temp_id", MOTOR_TEMP[BR-1], 3);
	    pnh.param("backleft_motor_temp_id", MOTOR_TEMP[BL-1], 4);
	    pnh.param("frontleft_controller_temp_id", CONTROLLER_TEMP[FL-1], 5);
	    pnh.param("frontright_controller_temp_id",CONTROLLER_TEMP[FR-1], 5);
	    pnh.param("backright_controller_temp_id", CONTROLLER_TEMP[BR-1], 6);
	    pnh.param("backleft_controller_temp_id", CONTROLLER_TEMP[BL-1], 7);
	    pnh.param("victor_temp_id", VICOR_TEMP, 7);
	    pnh.param("top_plate_temp_id", TOP_PLATE_TEMP, 8);
	    pnh.param("right_drive_motor_temp_id", RIGHT_DRIVE_TEMP, 9);
	    pnh.param("left_drive_motor_temp_id", LEFT_DRIVE_TEMP, 10);
	    pnh.param("front_cam_led_id", FRONT_CAM_LED, 1);
	    pnh.param("bottom_cam_led_id", BOTTOM_CAM_LED, 2);
	    pnh.param("back_cam_led_id", BACK_CAM_LED, 3);
	    pnh.param("backup_batt_voltage_id", BACKUP_BATT_VOLTAGE, 1);
	    
	    pnh.param("frontleft_pod_neutral_position", POD_POSITION_NEUTRAL[FL-1], 0.0);
	    pnh.param("frontright_pod_neutral_position", POD_POSITION_NEUTRAL[FR-1], 0.0);
	    pnh.param("backright_pod_neutral_position", POD_POSITION_NEUTRAL[BR-1], 0.0);
	    pnh.param("backleft_pod_neutral_position", POD_POSITION_NEUTRAL[BL-1], 0.0);
	    pnh.param("frontleft_pod_reverse", POD_REV[FL-1], false);
	    pnh.param("frontright_pod_reverse", POD_REV[FR-1], false);
	    pnh.param("backright_pod_reverse", POD_REV[BR-1], false);
	    pnh.param("backleft_pod_reverse", POD_REV[BL-1], false);
	    
	    pnh.param<string>("water_sense_1_position", WATER_SENSE_POSITION[0], "Position 1");
		pnh.param<string>("water_sense_2_position", WATER_SENSE_POSITION[1], "Position 2");
		pnh.param<string>("water_sense_3_position", WATER_SENSE_POSITION[2], "Position 3");
		pnh.param<string>("water_sense_4_position", WATER_SENSE_POSITION[3], "Position 4");
		pnh.param<string>("water_sense_5_position", WATER_SENSE_POSITION[4], "Position 5");
		pnh.param<string>("water_sense_6_position", WATER_SENSE_POSITION[5], "Position 6");
		
	    pnh.param("top_plate_temp_warn_above", TOP_PLATE_TEMP_WARN_ABOVE, -1.0);
	    pnh.param("top_plate_temp_error_above", TOP_PLATE_TEMP_ERROR_ABOVE, -1.0);
	    pnh.param("abmient_temp_warn_above", AMBIENT_TEMP_WARN_ABOVE, -1.0);
	    pnh.param("ambient_temp_error_above", AMBIENT_TEMP_ERROR_ABOVE, -1.0);
	    pnh.param("pressure_warn_below", PRESSURE_WARN_BELOW, -1.0);
	    pnh.param("pressure_error_below", PRESSURE_ERROR_BELOW, -1.0);
	    pnh.param("humidity_warn_above", HUMIDITY_WARN_ABOVE, -1.0);
	    pnh.param("humidity_error_above", HUMIDITY_ERROR_ABOVE, -1.0);
	    
	    pnh.param("pod_motor_temp_warn_above", POD_MOTOR_TEMP_WARN_ABOVE, -1.0);
	    pnh.param("pod_motor_temp_error_above", POD_MOTOR_TEMP_ERROR_ABOVE, -1.0);
	    pnh.param("pod_controller_temp_warn_above", POD_CONTROLLER_TEMP_WARN_ABOVE, -1.0);
	    pnh.param("pod_controller_temp_error_above", POD_CONTROLLER_TEMP_ERROR_ABOVE, -1.0);
	    pnh.param("pod_motor_current_warn_above", POD_MOTOR_CURRENT_WARN_ABOVE, -1.0);
	    pnh.param("pod_motor_current_error_above", POD_MOTOR_CURRENT_ERROR_ABOVE, -1.0);
	    
	    pnh.param("feedback_timeout", HS_FEEDBACK_TIMEOUT, 250);
	    
	    pnh.param("main_batt_voltage_warn_below", MAIN_BATT_VOLTAGE_WARN_BELOW, -1.0);
	    pnh.param("main_batt_charge_warn_below", MAIN_BATT_CHARGE_WARN_BELOW, -1.0);
	    pnh.param("main_batt_temp_warn_above", MAIN_BATT_TEMP_WARN_ABOVE, -1.0);
	    
	    pnh.param("vicor_temp_warn_above", VICOR_TEMP_WARN_ABOVE, -1.0);
	    pnh.param("vicor_temp_error_above", VICOR_TEMP_ERROR_ABOVE, -1.0);
	    pnh.param("backup_batt_voltage_warn_below", BACKUP_BATT_VOLTAGE_WARN_BELOW, -1.0);
	    pnh.param("main_voltage_warn_below", MAIN_VOLTAGE_WARN_BELOW, -1.0);
	    pnh.param("main_charge_warn_below", MAIN_CHARGE_WARN_BELOW, -1.0);
	    
	    pnh.param("drive_motor_temp_warn_above", DRIVE_MOTOR_TEMP_WARN_ABOVE, -1.0);
	    pnh.param("drive_motor_temp_error_above", DRIVE_MOTOR_TEMP_ERROR_ABOVE, -1.0);
	    
	  
	    host_enabled = false;
	    main_board_status_level = diagnostic_msgs::DiagnosticStatus::ERROR;
	    main_board_status = "Not Connected";
	    main_board_connected = false;
	    board_enabled = false;
	    for (int l = 0; l < 4; l++)
	        pod_force_disable[l] = false;
	    
	    nh.createTimer(ros::Duration(1), &MainBoardDriver::heartbeat_callback, this);
		
		//Setup pod actuator interfaces
		hardware_interface::ActuatorStateHandle state_handle0("walrus/front_left_pod_joint_actuator", &pod_position[FL-1], &pod_velocity[FL-1], &pod_effort[FL-1]);
		asi.registerHandle(state_handle0);
		hardware_interface::ActuatorStateHandle state_handle1("walrus/front_right_pod_joint_actuator", &pod_position[FR-1], &pod_velocity[FR-1], &pod_effort[FR-1]);
		asi.registerHandle(state_handle1);
		hardware_interface::ActuatorStateHandle state_handle2("walrus/back_right_pod_joint_actuator", &pod_position[BR-1], &pod_velocity[BR-1], &pod_effort[BR-1]);
		asi.registerHandle(state_handle2);
		hardware_interface::ActuatorStateHandle state_handle3("walrus/back_left_pod_joint_actuator", &pod_position[BL-1], &pod_velocity[BL-1], &pod_effort[BL-1]);
		asi.registerHandle(state_handle3);		
		hardware_interface::ActuatorHandle effort_handle0(state_handle0, &pod_effort_cmd[FL-1]);
		aei.registerHandle(effort_handle0);
		hardware_interface::ActuatorHandle effort_handle1(state_handle1, &pod_effort_cmd[FR-1]);
		aei.registerHandle(effort_handle1);
		hardware_interface::ActuatorHandle effort_handle2(state_handle2, &pod_effort_cmd[BR-1]);
		aei.registerHandle(effort_handle2);
		hardware_interface::ActuatorHandle effort_handle3(state_handle3, &pod_effort_cmd[BL-1]);
		aei.registerHandle(effort_handle3);
		
		//Add diagnostics updaters
		diagnostic_updater.setHardwareID("Walrus Main Board");
		diagnostic_updater.add("Chassis", this, &MainBoardDriver::chassis_diagnostic_callback);
		diagnostic_updater.add("Front Left Pod", boost::bind(&MainBoardDriver::pod_diagnostic_callback, this, _1, FL-1));
		diagnostic_updater.add("Front Right Pod", boost::bind(&MainBoardDriver::pod_diagnostic_callback, this, _1, FR-1));
		diagnostic_updater.add("Back Right Pod", boost::bind(&MainBoardDriver::pod_diagnostic_callback, this, _1, BR-1));
		diagnostic_updater.add("Back Left Pod", boost::bind(&MainBoardDriver::pod_diagnostic_callback, this, _1, BL-1));
		diagnostic_updater.add("Main Control Board", this, &MainBoardDriver::mainboard_diagnostic_callback);
		diagnostic_updater.add("Battery 1", boost::bind(&MainBoardDriver::batt_diagnostic_callback, this, _1, 0));
		diagnostic_updater.add("Battery 2", boost::bind(&MainBoardDriver::batt_diagnostic_callback, this, _1, 1));
		diagnostic_updater.add("Battery 3", boost::bind(&MainBoardDriver::batt_diagnostic_callback, this, _1, 2));
		diagnostic_updater.add("Battery 4", boost::bind(&MainBoardDriver::batt_diagnostic_callback, this, _1, 3));
		diagnostic_updater.add("Power Systems", this, &MainBoardDriver::power_diagnostic_callback);
		diagnostic_updater.add("Left Drive Motor Temperature", this, &MainBoardDriver::left_drive_temp_diagnostic_callback);
		diagnostic_updater.add("Right Drive Motor Temperature", this, &MainBoardDriver::right_drive_temp_diagnsotic_callback);
	}
	
	bool MainBoardDriver::init()
	{	    
	    //Setup publishers and subscribers to communicate with the embedded board
		hs_control = nh.advertise<walrus_firmware_msgs::MainBoardHighSpeedControl>("/walrus/main_board/hs_control", 1000);
		to_board = nh.advertise<walrus_firmware_msgs::MainBoardControl>("/walrus/main_board/PC_to_board_control", 1000);
		hs_feedback = nh.subscribe("/walrus/main_board/hs_feedback", 1000, &MainBoardDriver::hs_feedback_callback, this);
		ls_data = nh.subscribe("/walrus/main_board/ls_data", 1000, &MainBoardDriver::ls_data_callback, this);
		from_board = nh.subscribe("/walrus/main_board/board_to_PC_control", 1000, &MainBoardDriver::from_board_callback, this);
	    return true;
	}
	
	void MainBoardDriver::read()
	{
        double temp, dr;
	    double interval = (ros::Time::now() - last_read).toSec();
	    last_read = ros::Time::now();
	    
	    {
	        boost::lock_guard<boost::mutex> lock(control_data_mutex);
	       
	        for (int l = 0; l < 4; l++)
	        {
		        temp = (hs_feedback_msg.pod_position[l]/1024.0)*2*PI;
		        if (POD_REV[l])
		            temp = (2*PI) - temp;
		        temp -= POD_POSITION_NEUTRAL[l];
		        if (temp < 0)
		            temp += 2*PI;
		        dr = temp - pod_position[l];
		        if (dr > 0 && dr > PI) //Decrease angle across 0
		            pod_velocity[l] = (dr-2*PI)/interval;
		        else if (dr < 0 && dr < -PI) //Increase angle across 0
		            pod_velocity[l] = (dr+2*PI)/interval;
		        else  
		            pod_velocity[l] = dr/interval;
		        pod_position[l] = temp;		
		        pod_effort[l] = (hs_feedback_msg.motor_current[l] / 1000.0) * 29.184; //RS550 motors with 5120:1 reduction outputs 29.185 Nm/A
	        }		
	    }

	}
	
	void MainBoardDriver::write()
	{			
	    walrus_firmware_msgs::MainBoardHighSpeedControl hs_control_msg;    
	    {
	        boost::lock_guard<boost::mutex> lock(control_data_mutex);
	        //Send motor control message	        
	        for (int l = 0; l < 4; l++)
	        {
	            pod_power[l] = (pod_effort_cmd[l]*250) + 1500;
		        hs_control_msg.motor_power[l] = pod_power[l];
		    }
		}
		
		hs_control.publish(hs_control_msg);		
	}
	
	void MainBoardDriver::update_diagnostics()
	{
		diagnostic_updater.update();
	}
	
	//Subscription callbacks
	void MainBoardDriver::hs_feedback_callback(const walrus_firmware_msgs::MainBoardHighSpeedFeedback& msg)
	{
	    boost::lock_guard<boost::mutex> lock(control_data_mutex);
	    hs_feedback_msg = msg;
	    last_hs_msg = ros::Time::now();
	}
	void MainBoardDriver::ls_data_callback(const walrus_firmware_msgs::MainBoardLowSpeedData& msg)
	{
	    boost::lock_guard<boost::mutex> lock(sensor_data_mutex);
        
        //Temp sensors
        for (int l = 0; l < 10; l++)
            temp_sensors[l] = msg.temp_sense[l] / 10.0;
        
        //Ambient temp
        ambient_temp = msg.board_temp / 100.0;
        
        //Humidity
        humidity = msg.humidity / 100.0;
        
        //Pressure
        pressure = msg.pressure / 1000.0;
        
        //Water sensors
        water[0] = msg.water_sense & 1;
        water[1] = msg.water_sense & 2;
        water[2] = msg.water_sense & 4;
        water[3] = msg.water_sense & 8;
        water[4] = msg.water_sense & 16;
        water[5] = msg.water_sense & 32;
        
        //Backup battery
        backup_voltage = msg.tension[BACKUP_BATT_VOLTAGE];
        
        int count = 0;
        main_voltage = 0;
        main_current = 0;
        main_avg_current = 0;
        main_charge = 0;
        //Battery data
        for (int l = 0; l < 4; l++)
        {
            if (msg.lcell_charge[l] > 0)
            {
                //if it wasn't present, ask it for mfr and name data
                if (!batteries[l].present)
                {
                    batteries[l].mfr = "Not available";
                    batteries[l].dev_name = "Not available";
                    batteries[l].chemistry = "Not available";
                    walrus_firmware_msgs::MainBoardControl msg2;
                    msg2.type = walrus_firmware_msgs::MainBoardControl::REQ_BATT_INFO;
                    msg2.index = l;
                    msg2.value = 0; 
                    msg2.msg = "";
                    to_board.publish(msg2);
                }
                batteries[l].present = true;
                batteries[l].upper_voltage = msg.lcell_voltage[l] / 1000.0;
                batteries[l].upper_current = msg.lcell_current[l] / 1000.0;
                batteries[l].upper_charge = msg.lcell_charge[l] / 100.0;
                batteries[l].upper_temp = msg.lcell_temp[l] / 100.0;
                batteries[l].upper_avg_current = msg.lcell_avgcurr[l] / 1000.0;
                batteries[l].upper_voltage = msg.ucell_voltage[l] / 1000.0;
                batteries[l].upper_current = msg.ucell_current[l] / 1000.0;
                batteries[l].upper_charge = msg.ucell_charge[l] / 100.0;
                batteries[l].upper_temp = msg.ucell_temp[l] / 100.0;
                batteries[l].upper_avg_current = msg.ucell_avgcurr[l] / 1000.0;
                batteries[l].shutdown = (batteries[l].lower_voltage < 2 || batteries[l].upper_voltage < 2);
                batteries[l].combined_voltage = batteries[l].lower_voltage + batteries[l].upper_voltage;
                batteries[l].combined_current = (batteries[l].lower_current + batteries[l].upper_current) / 2;
                batteries[l].combined_avg_current = (batteries[l].lower_avg_current + batteries[l].upper_avg_current) / 2;
                batteries[l].combined_charge = (batteries[l].lower_charge + batteries[l].upper_charge) / 2;
                main_voltage += batteries[l].combined_voltage;
                main_current += batteries[l].combined_current;
                main_avg_current += batteries[l].combined_avg_current;
                main_charge += batteries[l].combined_charge;
                count++;
            }
            else
                batteries[l].present = false;
        }
        main_voltage /= count;
        main_current /= count;
        main_avg_current /= count;
        main_charge /= count;
    }
    
	void MainBoardDriver::from_board_callback(const walrus_firmware_msgs::MainBoardControl& msg)
	{
	    boost::lock_guard<boost::mutex> lock(sensor_data_mutex);
	    
	    switch (msg.type)
	    {
            case walrus_firmware_msgs::MainBoardControl::BATT_MFR:
                batteries[msg.index].mfr = msg.msg;
            break;
            case walrus_firmware_msgs::MainBoardControl::BATT_NAME:
                batteries[msg.index].dev_name = msg.msg;
            break;
            case walrus_firmware_msgs::MainBoardControl::BATT_CHEM:
                batteries[msg.index].chemistry = msg.msg;
            break; 
            case walrus_firmware_msgs::MainBoardControl::STATUS:
                main_board_status = msg.msg;
                main_board_status_level = msg.value;
            break;
	    }
    }
    
    //Diagnostics updaters
    void MainBoardDriver::chassis_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        boost::lock_guard<boost::mutex> lock(sensor_data_mutex);
    
        bool warning;
        bool error;
        string msg = "";
        if (main_board_connected)
        {
            //Check top plate temperature
            if (TOP_PLATE_TEMP_ERROR_ABOVE > 0 && temp_sensors[TOP_PLATE_TEMP-1] > TOP_PLATE_TEMP_ERROR_ABOVE)
            {
                error = true;
                msg += "Top plate temperature critical. ";
            }
            else if (TOP_PLATE_TEMP_WARN_ABOVE > 0 && temp_sensors[TOP_PLATE_TEMP-1] > TOP_PLATE_TEMP_WARN_ABOVE)
            {
                warning = true;
                msg += "Top plate temperature high. ";
            }
            
            //Check ambient temperature
            if (AMBIENT_TEMP_ERROR_ABOVE > 0 && ambient_temp > AMBIENT_TEMP_ERROR_ABOVE)
            {
                error = true;
                msg += "Internal temperature critical. ";
            }
            else if (AMBIENT_TEMP_WARN_ABOVE > 0 && ambient_temp > AMBIENT_TEMP_WARN_ABOVE)
            {
                warning = true;
                msg += "Internal temperature high. ";
            }
            
            //Check humidity
            if (HUMIDITY_ERROR_ABOVE > 0 && humidity > HUMIDITY_ERROR_ABOVE)
            {
                error = true;
                msg += "Internal humidity critical. ";
            }
            else if (HUMIDITY_WARN_ABOVE > 0 && humidity > HUMIDITY_WARN_ABOVE)
            {
                warning = true;
                msg += "Internal humidity high. ";
            }
            
            //Check pressure
            if (PRESSURE_ERROR_BELOW> 0 && ambient_temp < PRESSURE_ERROR_BELOW)
            {
                error = true;
                msg += "Internal pressure critical. ";
            }
            else if (PRESSURE_WARN_BELOW > 0 && ambient_temp < PRESSURE_WARN_BELOW)
            {
                warning = true;
                msg += "Internal pressure low. ";
            }
            
            //Check water
            int trip_count = 0;
            for (int l = 0; l < 6; l++)
            {
                if (water[l])
                    trip_count++;
            }
            if (trip_count > 0)
            {
                warning = true;
                msg += "Hull breach. ";
            }
            if (trip_count > 3)
                error = true;
            for (int l = 0; l < 6; l++)
            {
                if (water[l])
                    msg += "Water leak in " + WATER_SENSE_POSITION[l] + ". ";
            }
            
            uint8_t level = diagnostic_msgs::DiagnosticStatus::OK;
	        if (error)
	            level = diagnostic_msgs::DiagnosticStatus::ERROR;
	        else if (warning)
	            level = diagnostic_msgs::DiagnosticStatus::WARN;
	        else
	            msg = "Everything OK";
            
            stat.summary(level, msg);
        }
        else
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No Data");
            
        stat.add("Top Plate Temperature", main_board_connected ? boost::lexical_cast<string>(temp_sensors[TOP_PLATE_TEMP-1]) + "\xc2\xb0""C" : "No Data");
        stat.add("Internal Temperature", main_board_connected ? boost::lexical_cast<string>(ambient_temp) + "\xc2\xb0""C" : "No Data");
        stat.add("Internal Humidity", main_board_connected ? boost::lexical_cast<string>(humidity) + "%" : "No Data");
        stat.add("Internal Pressure", main_board_connected ? boost::lexical_cast<string>(pressure) + " kPa" : "No Data");
        for (int l = 0; l < 6; l++)
            stat.add("Water in " + WATER_SENSE_POSITION[l], main_board_connected ? (water[l] ? "Yes": "No") : "No Data");
    }
    
	void MainBoardDriver::pod_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat, int index)
	{
	    boost::lock_guard<boost::mutex> lock(sensor_data_mutex);
	
	    bool warning;
	    bool error;
	    string msg = "";   
	  
	    if (main_board_connected)
        {      
	        //Check over current
	        if (POD_MOTOR_CURRENT_ERROR_ABOVE > 0 && pod_current[index] > POD_MOTOR_CURRENT_ERROR_ABOVE)
	        {
	            error = true;
	            msg += "Motor current critical. ";
	            pod_force_disable[index] = true;
	        }
	        else if (POD_MOTOR_CURRENT_WARN_ABOVE > 0 && pod_current[index] > POD_MOTOR_CURRENT_WARN_ABOVE)
	        {
	            warning = true;
	            msg += "Motor current high. ";	        
	        }
	        
	        //Check motor over temperature
	        if (POD_MOTOR_TEMP_ERROR_ABOVE > 0 && temp_sensors[MOTOR_TEMP[index]-1] > POD_MOTOR_TEMP_ERROR_ABOVE)
	        {
	            error = true;
	            msg += "Motor temperature critical. ";
	            pod_force_disable[index] = true;
	        }
	        else if (POD_MOTOR_TEMP_WARN_ABOVE > 0 && temp_sensors[MOTOR_TEMP[index]-1] > POD_MOTOR_TEMP_WARN_ABOVE)
	        {
	            warning = true;
	            msg += "Motor temperature high. ";
	        }
	        
	        //Check controller over temperature
	        if (POD_CONTROLLER_TEMP_ERROR_ABOVE > 0 && temp_sensors[CONTROLLER_TEMP[index]-1] > POD_CONTROLLER_TEMP_ERROR_ABOVE)
	        {
	            error = true;
	            msg += "Motor controller temperature critical. ";
	            pod_force_disable[index] = true;
	            pod_force_disable[index] = true;
	        }
	        else if (POD_CONTROLLER_TEMP_WARN_ABOVE > 0 && temp_sensors[CONTROLLER_TEMP[index]-1] > POD_CONTROLLER_TEMP_WARN_ABOVE)
	        {
	            warning = true;
	            msg += "Motor controller temperature high. ";
	        }
	        
	        uint8_t level = diagnostic_msgs::DiagnosticStatus::OK;
	        if (error)
	            level = diagnostic_msgs::DiagnosticStatus::ERROR;
	        else if (warning)
	            level = diagnostic_msgs::DiagnosticStatus::WARN;
	        else
	            msg = "Everything OK";
	        
	        stat.summary(level, msg);
	    }
	    else
	        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No Data");
	    
	    stat.add("Velocity", main_board_connected ? boost::lexical_cast<string>(pod_velocity[index]) + " rpm" : "No Data");
	    stat.add("Position", main_board_connected ? boost::lexical_cast<string>((pod_position[index]*180/PI)) + "\xc2\xb0" : "No Data");
	    stat.add("Current", main_board_connected ? boost::lexical_cast<string>(pod_current[index]) + " A" : "No Data");
	    stat.add("Motor Temperature", main_board_connected ? boost::lexical_cast<string>(temp_sensors[MOTOR_TEMP[index]-1]) + "\xc2\xb0""C" : "No Data");
	    stat.add("Controller Temperature", main_board_connected ? boost::lexical_cast<string>(temp_sensors[CONTROLLER_TEMP[index]-1]) + "\xc2\xb0""C" : "No Data");
    }
    
	
	void MainBoardDriver::mainboard_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
	{
	    boost::lock_guard<boost::mutex> lock(sensor_data_mutex);
	    
	    {
	    boost::lock_guard<boost::mutex> lock(control_data_mutex);
	    double interval = (ros::Time::now() - last_hs_msg).toSec();
	    }
	        
	    stat.summary(main_board_status_level, main_board_status);
	    
        stat.add("Host output status", host_enabled ? "Enabled" : "Disabled");
        stat.add("Board output status", board_enabled ? "Enabled" : "Disabled");
    }
    
	void MainBoardDriver::batt_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat, int index)
	{
	    bool pres = batteries[index].present;
	    bool warning;
	    string msg = "";   
	    
	    if (main_board_connected)
        {    
	        if (pres)
	        { 
	            //check voltage
	            if (MAIN_BATT_VOLTAGE_WARN_BELOW > 0 && batteries[index].combined_voltage < MAIN_BATT_VOLTAGE_WARN_BELOW)
	            {
	                warning = true;
	                msg += "Voltage low. ";
	            }
	            //check charge
	            if (MAIN_BATT_CHARGE_WARN_BELOW > 0 && batteries[index].combined_charge < MAIN_BATT_CHARGE_WARN_BELOW)
	            {
	                warning = true;
	                msg += "Charge low. ";
	            }
	            //check temp of high cell
	            if (MAIN_BATT_TEMP_WARN_ABOVE > 0 && batteries[index].upper_temp > MAIN_BATT_TEMP_WARN_ABOVE)
	            {
	                warning = true;
	                msg += "Upper cell temperature high. ";
	            }
	            //check temp of low cell
	            if (MAIN_BATT_TEMP_WARN_ABOVE > 0 && batteries[index].lower_temp > MAIN_BATT_TEMP_WARN_ABOVE)
	            {
	                warning = true;
	                msg += "Lower cell temperature high. ";
	            }
	            
	            uint8_t level = diagnostic_msgs::DiagnosticStatus::OK;
	            if (warning)
	                level = diagnostic_msgs::DiagnosticStatus::WARN;
	            else
	                msg = "Everything OK";
	        }
	        else
	            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Not Present");
	    }
	    else
	    {
	        pres = false;
	        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No Data");
	    }
	    
	    stat.add("Present", pres ? "Yes" : "No");
	    stat.add("Charge", pres ? boost::lexical_cast<string>(batteries[index].combined_charge) + "%" : "No Data");
	    stat.add("Voltage", pres ? boost::lexical_cast<string>(batteries[index].combined_voltage) + " V" : "No Data");
	    stat.add("Current", pres ? boost::lexical_cast<string>(batteries[index].combined_current) + " A" : "No Data");
	    stat.add("Average Current", pres ? boost::lexical_cast<string>(batteries[index].combined_avg_current) + " A" : "No Data");
	    stat.add("Shutdown", pres ? (batteries[index].shutdown ? "Yes" : "No") : "No Data");
	    stat.add("Manufacturer", pres ? boost::lexical_cast<string>(batteries[index].mfr) : "No Data");
	    stat.add("Device Name", pres ? boost::lexical_cast<string>(batteries[index].dev_name) : "No Data");
	    stat.add("Battery Type", pres ? boost::lexical_cast<string>(batteries[index].chemistry) : "No Data");
	    stat.add("Lower Cell Voltage", pres ? boost::lexical_cast<string>(batteries[index].lower_voltage) + " V" : "No Data");
	    stat.add("Lower Cell Current", pres ? boost::lexical_cast<string>(batteries[index].lower_current) + " A" : "No Data");
	    stat.add("Lower Cell Average Current", pres ? boost::lexical_cast<string>(batteries[index].lower_avg_current) + " A" : "No Data");
	    stat.add("Lower Cell Charge", pres ? boost::lexical_cast<string>(batteries[index].lower_charge) + "%" : "No Data");
	    stat.add("Lower Cell Temperature", pres ? boost::lexical_cast<string>(batteries[index].lower_temp) + "\xc2\xb0""C" : "No Data");
	    stat.add("Upper Cell Voltage", pres ? boost::lexical_cast<string>(batteries[index].upper_voltage) + " V" : "No Data");
	    stat.add("Upper Cell Current", pres ? boost::lexical_cast<string>(batteries[index].upper_current) + " A" : "No Data");
	    stat.add("Upper Cell Average Current", pres ? boost::lexical_cast<string>(batteries[index].upper_avg_current) + " A" : "No Data");
	    stat.add("Upper Cell Charge", pres ? boost::lexical_cast<string>(batteries[index].upper_charge) + "%" : "No Data");
	    stat.add("Upper Cell Temperature", pres ? boost::lexical_cast<string>(batteries[index].upper_temp) + "\xc2\xb0""C" : "No Data");
	}
	
	void MainBoardDriver::power_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
	{
	    bool warning;
	    bool error;
	    string msg = "";   
	    
	    if (main_board_connected)
        {    
	        //Check charge
	        if (MAIN_CHARGE_WARN_BELOW > 0 && main_charge < MAIN_CHARGE_WARN_BELOW)
	        {
	            warning = true;
	            msg += "Main battery low. ";
	        }
	        
	        //Check main voltage
	        if (MAIN_VOLTAGE_WARN_BELOW > 0 && main_voltage < MAIN_VOLTAGE_WARN_BELOW)
	        {
	            warning = true;
	            msg += "Main battery voltage low. ";
	        }
	        
	        //Check backup voltage
	        if (BACKUP_BATT_VOLTAGE_WARN_BELOW > 0 && backup_voltage < BACKUP_BATT_VOLTAGE_WARN_BELOW)
	        {
	            warning = true;
	            if (backup_voltage < 1)
	                msg += "Backup battery not present. ";
	            else
	                msg += "Backup battery voltage low. ";
	        }
	        
	        //Check vicor temperature
	         if (VICOR_TEMP_ERROR_ABOVE > 0 && temp_sensors[VICOR_TEMP-1] > VICOR_TEMP_ERROR_ABOVE)
	        {
	            error = true;
	            msg += "Vicor temperature critical. ";
	        }
	        else if (VICOR_TEMP_WARN_ABOVE > 0 && temp_sensors[VICOR_TEMP-1] > VICOR_TEMP_WARN_ABOVE)
	        {
	            warning = true;
	            msg += "Vicor temperature high. ";
	        }
	        
	        uint8_t level = diagnostic_msgs::DiagnosticStatus::OK;
	        if (error)
	            level = diagnostic_msgs::DiagnosticStatus::ERROR;
	        else if (warning)
	            level = diagnostic_msgs::DiagnosticStatus::WARN;
	        else
	            msg = "Everything OK";
	            
	        stat.summary(level, msg);
	    }
	    else
	        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No Data");
	    
	    
	    stat.add("Main Battery Charge", main_board_connected ? boost::lexical_cast<string>(main_charge) + "%" : "No Data");
	    stat.add("Main Battery Voltage", main_board_connected ? boost::lexical_cast<string>(main_voltage) + " V" : "No Data");
	    stat.add("Main Battery Current", main_board_connected ? boost::lexical_cast<string>(main_current) + " A" : "No Data");
	    stat.add("Main Batter Average Current", main_board_connected ? boost::lexical_cast<string>(main_avg_current) + " A" : "No Data");
	    stat.add("Backup Battery Voltage", main_board_connected ? boost::lexical_cast<string>(backup_voltage) + " V" : "No Data");
	    stat.add("Vicor Temperature", main_board_connected ? boost::lexical_cast<string>(temp_sensors[VICOR_TEMP-1]) + "\xc2\xb0""C" : "No Data");
	}
	
	void MainBoardDriver::left_drive_temp_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
	{	    
	    if (main_board_connected)
        {    
	        if (DRIVE_MOTOR_TEMP_ERROR_ABOVE > 0 && temp_sensors[LEFT_DRIVE_TEMP-1] > DRIVE_MOTOR_TEMP_ERROR_ABOVE)
	            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Temperature critical.");
	        else if (DRIVE_MOTOR_TEMP_WARN_ABOVE > 0 && temp_sensors[LEFT_DRIVE_TEMP-1] > DRIVE_MOTOR_TEMP_WARN_ABOVE)
                stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Temperature high.");
	        else
	            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Temperature OK.");
	    }
	    else
	        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No Data");
	        
	    stat.add("Temperature", main_board_connected ? boost::lexical_cast<string>(temp_sensors[LEFT_DRIVE_TEMP-1]) + "\xc2\xb0""C" : "No Data");
    }
    
    void MainBoardDriver::right_drive_temp_diagnsotic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        if (main_board_connected)
        {
            if (DRIVE_MOTOR_TEMP_ERROR_ABOVE > 0 && temp_sensors[RIGHT_DRIVE_TEMP-1] > DRIVE_MOTOR_TEMP_ERROR_ABOVE)
	            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Temperature critical.");
	        else if (DRIVE_MOTOR_TEMP_WARN_ABOVE > 0 && temp_sensors[RIGHT_DRIVE_TEMP-1] > DRIVE_MOTOR_TEMP_WARN_ABOVE)
                stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Temperature high.");
	        else
	            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Temperature OK.");
	    }
	    else
	        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No Data");
	        
	    stat.add("Temperature", main_board_connected ? boost::lexical_cast<string>(temp_sensors[RIGHT_DRIVE_TEMP-1]) + "\xc2\xb0""C" : "No Data");
    }
    
    void MainBoardDriver::heartbeat_callback(const ros::TimerEvent&)
    {
        walrus_firmware_msgs::MainBoardControl msg;
        msg.type = walrus_firmware_msgs::MainBoardControl::KEEP_ALIVE;
        msg.index = 0;
        msg.value = 0; 
        msg.msg = "";
        to_board.publish(msg);     
    }

}
