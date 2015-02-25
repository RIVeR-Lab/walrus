#include "walrus_mainboard_driver/walrus_mainboard_driver.h"

#define PI 3.14159265358979323846

namespace walrus_mainboard_driver
{

	MainBoardDriver::MainBoardDriver(hardware_interface::ActuatorStateInterface& asi,
				  hardware_interface::EffortActuatorInterface &aei,
				  ros::NodeHandle& nh, ros::NodeHandle& pnh)
	: asi_(asi), aei_(aei), diagnostic_updater(nh, pnh), nh(nh), pnh(pnh)
	{	    
	    //Setup publishers and subscribers to communicate with the embedded board
		hs_control = nh.advertise<walrus_firmware_msgs::MainBoardHighSpeedControl>("/walrus/main_board/hs_control", 1000);
		to_board = nh.advertise<walrus_firmware_msgs::MainBoardControl>("/walrus/main_board/PC_to_board_control", 1000);
		hs_feedback = nh.subscribe("/walrus/main_board/hs_feedback", 1000, &MainBoardDriver::hs_feedback_callback, this);
		ls_control = nh.subscribe("/walrus/main_board/ls_data", 1000, &MainBoardDriver::ls_data_callback, this);
		from_board = nh.subscribe("/walrus/main_board/board_to_PC_control", 1000, &MainBoardDriver::from_board_callback, this);
		
		//Setup pod actuator interfaces
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
		
		//Add diagnostics updaters
		diagnostic_updater.add("Chassis", this, &MainBoardDriver::chassis_diagnostic_callback);
		diagnostic_updater.add("Front Left Pod", this, &MainBoardDriver::FLPod_diagnostic_callback);
		diagnostic_updater.add("Front Right Pod", this, &MainBoardDriver::FRPod_diagnostic_callback);
		diagnostic_updater.add("Back Right Pod", this, &MainBoardDriver::BRPod_diagnostic_callback);
		diagnostic_updater.add("Back Left Pod", this, &MainBoardDriver::BLPod_diagnostic_callback);
		diagnostic_updater.add("Main Control Board", this, &MainBoardDriver::mainboard_diagnostic_callback);
		diagnostic_updater.add("Battery 1", boost::bind(&MainBoardDriver::batt1_diagnostic_callback, this, _1, 0);
		diagnostic_updater.add("Battery 2", boost::bind(&MainBoardDriver::batt1_diagnostic_callback, this, _1, 1);
		diagnostic_updater.add("Battery 3", boost::bind(&MainBoardDriver::batt1_diagnostic_callback, this, _1, 2);
		diagnostic_updater.add("Battery 4", boost::bind(&MainBoardDriver::batt1_diagnostic_callback, this, _1, 3);
		diagnostic_updater.add("Power Systems", this, &MainBoardDriver::power_diagnostic_callback);
		diagnostic_updater.add("Left Drive Motor Temperature", this, &MainBoardDriver::left_drive_temp_diagnostic_callback);
		diagnostic_updater.add("Right Drive Motor Temperature", this, &MainBoardDriver::right_drive_temp_diagnsotic_callback);
	}
	
	bool MainBoardDriver::init()
	{
	    //Resolve pod names to board IDs
	    nh.param("frontleft_pod_id", FL, 1);
	    nh.param("frontright_pod_id", FR, 2);
	    nh.param("backright_pod_id", BR, 3);
	    nh.param("backleft_pod_id", BL, 4);
	    nh.param("frontleft_motor_temp_id", FL_MOTOR_TEMP, 1);
	    nh.param("frontright_motor_temp_id", FR_MOTOR_TEMP, 2);
	    nh.param("backright_motor_temp_id", BR_MOTOR_TEMP, 3);
	    nh.param("backleft_motor_temp_id", BL_MOTOR_TEMP, 4);
	    nh.param("frontleft_controller_temp_id", FL_CONTROLLER_TEMP, 5);
	    nh.param("frontright_controller_temp_id", FR_CONTROLLER_TEMP, 5);
	    nh.param("backright_controller_temp_id", BR_CONTROLLER_TEMP, 6);
	    nh.param("backleft_controller_temp_id", BL_CONTROLLER_TEMP, 7);
	    nh.param("victor_temp_id", VICOR_TEMP, 7);
	    nh.param("top_plate_temp_id", TOP_PLATE_TEMP, 8);
	    nh.param("right_drive_motor_temp_id", RIGHT_DRIVE_TEMP, 9);
	    nh.param("left_drive_motor_temp_id", LEFT_DRIVE_TEMP, 10);
	    nh.param("front_cam_led_id", FRONT_CAM_LED, 1);
	    nh.param("bottom_cam_led_id", BOTTOM_CAM_LED, 2);
	    nh.param("back_cam_led_id", BACK_CAM_LED, 3);
	    nh.param("backup_batt_voltage_id", BACKUP_BATT_VOLTAGE, 1);
	    nh.param("pod_motor_temp_warn_above", POD_MOTOR_TEMP_WARN_ABOVE, 75);
	    nh.param("pod_motor_temp_error_above", POD_MOTOR_TEMP_ERROR_ABOVE, -1);
	    nh.param("pod_controller_temp_warn_above", POD_CONTROLLER_TEMP_WARN_ABOVE, -1);
	    nh.param("pod_controller_temp_error_above", POD_CONTROLLER_TEMP_ERROR_ABOVE, -1);
	    nh.param("pod_motor_current_warn_above", POD_MOTOR_CURRENT_WARN_ABOVE, -1);
	    nh.param("pod_motor_current_error_above", POD_MOTOR_CURRENT_ERROR_ABOVE, -1);
	    nh.param("vicor_temp_warn_above", VICOR_TEMP_WARN_ABOVE, -1);
	    nh.param("vicor_temp_error_above", VICOR_TEMP_ERROR_ABOVE, -1);
	    nh.param("top_plate_temp_warn_above", TOP_PLATE_TEMP_WARN_ABOVE, -1);
	    nh.param("top_plate_temp_error_above", TOP_PLATE_TEMP_ERROR_ABOVE, -1);
	    nh.param("backup_batt_voltage_warn_below", BACKUP_BATT_VOLTAGE_WARN_BELOW, -1);
	    nh.param("main_voltage_warn_below", MAIN_VOLTAGE_WARN_BELOW, -1);
	    nh.param("main_charge_warn_below", MAIN_CHARGE_WARN_BELOW, -1);
	    nh.param("main_batt_voltage_warn_below", MAIN_BATT_VOLTAGE_WARN_BELOW, -1);
	    nh.param("main_batt_currnet_warn_below", MAIN_BATT_CURRENT_WARN_ABOVE, -1);
	    nh.param("main_batt_charge_warn_below", MAIN_BATT_CHARGE_WARN_BELOW, -1);
	    nh.param("main_batt_temp_warn_above", MAIN_BATT_TEMP_WARN_ABOVE, -1);
	    nh.param("drive_motor_temp_warn_above", DRIVE_MOTOR_TEMP_WARN_ABOVE, -1);
	    nh.param("drive_motor_temp_error_above", DRIVE_MOTOR_TEMP_ERROR_ABOVE, -1);
	    nh.param("abmient_temp_warn_above", AMBIENT_TEMP_WARN_ABOVE, -1);
	    nh.param("ambient_temp_error_above", AMBIENT_TEMP_ERROR_ABOVE, -1);
	    nh.param("pressure_warn_below", PRESSURE_WARN_BELOW, -1);
	    nh.param("pressure_error_below", PRESSURE_ERROR_BELOW, -1);
	    nh.param("humidity_warn_above", HUMIDITY_WARN_ABOVE, -1);
	    nh.param("humidity_error_above", HUMIDITY_ERROR_ABOVE, -1);
	    nh.param("water_sense_1_position", WATER_SENSE_POSITION[0], "Position 1");
		nh.param("water_sense_2_position", WATER_SENSE_POSITION[1], "Position 2");
		nh.param("water_sense_3_position", WATER_SENSE_POSITION[2], "Position 3");
		nh.param("water_sense_4_position", WATER_SENSE_POSITION[3], "Position 4");
		nh.param("water_sense_5_position", WATER_SENSE_POSITION[4], "Position 5");
		nh.param("water_sense_6_position", WATER_SENSE_POSITION[5], "Position 6");
	    
	    enabled = false;
	    FLPod_force_disable = false;
	    FRPod_force_disable = false;
	    BRPod_force_disable = false;
	    BLPod_force_disable = false;
	    
	    nh.createTimer(ros::Duration(1), &MainBoardDriver::heartbeat_callback, this);
	}
	
	void MainBoardDriver::read()
	{
	    control_data_mutex.lock();
	    
		FLPod_velocity = 0;
		FLPod_position = hs_feedback.pod_position[FL-1];
		FLPod_effort = hs_feedback.motor_current[FL-1];
		FRPod_velocity = 0;
		FRPod_position = hs_feedback.pod_position[FR-1];
		FRPod_effort = hs_feedback.motor_current[FR-1];
		BRPod_velocity = 0;
		BRPod_position = hs_feedback.pod_position[BR-1];
		BRPod_effort = hs_feedback.motor_current[BR-1];
		BLPod_velocity = 0;
		BLPod_position = hs_feedback.pod_position[BL-1];
		BLPod_effort = hs_feedback.motor_current[BL-1];
		
		control_data_mutex.unlock();
	}
	
	void MainBoardDriver::write()
	{		
	    //Convert effor to power
	    
	    //Send motor control message
	    walrus_firmware_msgs::MainBoardHighSpeedControl hs_control;
		hs_control.motor_power[FL-1] = FLPod_power;
		hs_control.motor_power[FR-1] = FRPod_power;
		hs_control.motor_power[BR-1] = BRPod_power;
		hs_control.motor_power[BL-1] = BLPod_power;
		tx.publish(hs_control);
	}
	
	void MainBoardDriver::update_diagnostics()
	{
		diagnostic_updater.update();
	}
	
	//Subscription callbacks
	void MainBoardDriver::hs_feedback_callback(const walrus_firmware_msgs::MainBoardHighSpeedFeedback& msg)
	{
	    control_data_mutex.lock();
	    hs_feedback = msg;
	    last_hs_msg = ros::Time::now();
	    control_data_mutex.unlock();
	}
	void MainBoardDriver::ls_data_callback(const walrus_firmware_msgs::MainBoardLowSpeedDatak& msg)
	{
	    sensor_data_mutex.lock();
        
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
        water[0] = water & 1;
        water[1] = water & 2;
        water[2] = water & 4;
        water[3] = water & 8;
        water[4] = water & 16;
        water[5] = water & 32;
        
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
            if (msg.lcell_charge > 0)
            {
                //if it wasn't present, ask it for mfr and name data
                if (!batteries[l].present)
                {
                    batteries[l].mfr = "Not available";
                    batteries[l].dev_name = "Not available";
                    batteries[l].chem = "Not available";
                    warlus_firmware_msgs::MainBoardControl msg2;
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
                batteries[l].upper_avg_current = msg.lcell_avgcurr[l] / 1000.0
                batteries[l].upper_voltage = msg.ucell_voltage[l] / 1000.0;
                batteries[l].upper_current = msg.ucell_current[l] / 1000.0;
                batteries[l].upper_charge = msg.ucell_charge[l] / 100.0;
                batteries[l].upper_temp = msg.ucell_temp[l] / 100.0;
                batteries[l].upper_avg_current = msg.ucell_avgcurr[l] / 1000.0
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

	    sensor_data_mutex.unlock();
    }
	void MainBoardDriver::from_board_callback(const walrus_firmware_msgs::MainBoardControl& msg)
	{
	    switch (msg.type)
	    {
            case walrus_firmware_msgs::MainBoardControl::BATT_MFR:
                sensor_data_mutex.lock();
                batteries[msg.index].mfr = msg.msg;
                sensor_data_mutex.unlock();
            break;
            case walrus_firmware_msgs::MainBoardControl::BATT_NAME:
                sensor_data_mutex.lock();
                batteries[msg.index].dev_name = msg.msg;
                sensor_data_mutex.unlock();
            break;
            case walrus_firmware_msgs::MainBoardControl::BATT_CHEM:
                sensor_data_mutex.lock();
                batteries[msg.index].chem = msg.msg;
                sensor_data_mutex.unlock();
            break; 
            case walrus_firmware_msgs::MainBoardControl::STATUS:
                sensor_data_mutex.lock();
                main_board_status = msg.msg;
                main_board_status_level = msg.value;
                sensor_data_mutex.unlock();
            break;
	    }
    }
    
    //Diagnostics updaters
    void MainBoardDriver::chassis_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        sensor_data_mutex.lock();
    
        bool warning;
        bool error;
        string msg = "";
        
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
        if (PRESSURE_ERROR_ABOVE > 0 && ambient_temp > PRESSURE_ERROR_ABOVE)
        {
            error = true;
            msg += "Internal pressure critical. ";
        }
        else if (PRESSURE_WARN_ABOVE > 0 && ambient_temp > PRESSURE_WARN_ABOVE)
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
                msg += "Water leak in " + WATER_SENSE_POSITION[l] + ". ");
        }
        
        uint8_t level = diagnostic_msgs::DiagnosticStatus::OK;
	    if (error)
	        level = diagnostic_msgs::DiagnosticStatus::ERROR;
	    else if (warning)
	        level = diagnostic_msgs::DiagnosticStatus::WARN;
	    else
	        msg = "Everything OK";
        
        stat.summary(level, msg);
        stat.add("Top Plate Temperature", temp_sensors[TOP_PLATE_TEMP-1] + "\xc2\xb0""C");
        stat.add("Internal Temperature", ambient_temp + "\xc2\xb0""C");
        stat.add("Internal Humidity", humidity + "%");
        stat.add("Internal Pressure", pressure + " kPa");
        for (int l = 0; l < 6; l++)
            stat.add("Water in " + WATER_SENSE_POSITION[l], water[l] ? "Yes": "No");
            
        sensor_data_mutex.unlock();
    }
    
	void MainBoardDriver::FLPod_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
	{
	    sensor_data_mutex.lock();
	
	    bool warning;
	    bool error;
	    string msg = "";   
	    
	    //Check over current
	    if (POD_MOTOR_CURRENT_ERROR_ABOVE > 0 && FLPod_current > POD_MOTOR_CURRENT_ERROR_ABOVE)
	    {
	        error = true;
	        msg += "Motor current critical. ";
	        FLPod_force_disable = true;
	    }
	    else if (POD_MOTOR_CURRENT_WARN_ABOVE > 0 && FLPod_current > POD_MOTOR_CURRENT_WARN_ABOVE)
	    {
	        warning = true;
	        msg += "Motor current high. ";	        
	    }
	    
	    //Check motor over temperature
	    if (POD_MOTOR_TEMP_ERROR_ABOVE > 0 && temp_sensors[FL_MOTOR_TEMP-1] > POD_MOTOR_TEMP_ERROR_ABOVE)
	    {
	        error = true;
	        msg += "Motor temperature critical. ";
	        FLPod_force_disable = true;
	    }
	    else if (POD_MOTOR_TEMP_WARN_ABOVE > 0 && temp_sensors[FL_MOTOR_TEMP-1] > POD_MOTOR_TEMP_WARN_ABOVE)
	    {
	        warning = true;
	        msg += "Motor temperature high. ";
	    }
	    
	    //Check controller over temperature
	    if (POD_CONTROLLER_TEMP_ERROR_ABOVE > 0 && temp_sensors[FL_CONTROLLER_TEMP-1] POD_CONTROLLER_TEMP_ERROR_ABOVE)
	    {
	        error = true;
	        msg += "Motor controller temperature critical. ";
	        FLPod_force_disable = true;
	        FRPod_force_disable = true;
	    }
	    else if (POD_CONTROLLER_TEMP_WARN_ABOVE > 0 && temp_sensors[FL_CONTROLLER_TEMP-1] > POD_CONTROLLER_TEMP_WARN_ABOVE)
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
	    
	    stat.add("Velocity", FLPod_velocity + " rpm");
	    stat.add("Position", (FLPod_position*180/PI) + "\xc2\xb0");
	    stat.add("Current", FLPod_current + " A");
	    stat.add("Motor Temperature", temp_sensors[FL_MOTOR_TEMP-1] + "\xc2\xb0""C");
	    stat.add("Controller Temperature", temp_sensors[FL_CONTROLLER_TEMP-1] + "\xc2\xb0""C");
	    
	    sensor_data_mutex.unlock();
    }
    
	void MainBoardDriver::FRPod_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
	{
	    sensor_data_mutex.lock();
	
	    bool warning;
	    bool error;
	    string msg = "";   
	    
	    //Check over current
	    if (POD_MOTOR_CURRENT_ERROR_ABOVE > 0 && FRPod_current > POD_MOTOR_CURRENT_ERROR_ABOVE)
	    {
	        error = true;
	        msg += "Motor current critical. ";
	        FRPod_force_disable = true;
	    }
	    else if (POD_MOTOR_CURRENT_WARN_ABOVE > 0 && FRPod_current > POD_MOTOR_CURRENT_WARN_ABOVE)
	    {
	        warning = true;
	        msg += "Motor current high. ";	        
	    }
	    
	    //Check motor over temperature
	    if (POD_MOTOR_TEMP_ERROR_ABOVE > 0 && temp_sensors[FR_MOTOR_TEMP-1] > POD_MOTOR_TEMP_ERROR_ABOVE)
	    {
	        error = true;
	        msg += "Motor temperature critical. ";
	        FRPod_force_disable = true;
	    }
	    else if (POD_MOTOR_TEMP_WARN_ABOVE > 0 && temp_sensors[FR_MOTOR_TEMP-1] > POD_MOTOR_TEMP_WARN_ABOVE)
	    {
	        warning = true;
	        msg += "Motor temperature high. ";
	    }
	    
	    //Check controller over temperature
	    if (POD_CONTROLLER_TEMP_ERROR_ABOVE > 0 && temp_sensors[FR_CONTROLLER_TEMP-1] POD_CONTROLLER_TEMP_ERROR_ABOVE)
	    {
	        error = true;
	        msg += "Motor controller temperature critical. ";
	        FRPod_force_disable = true;
	        FRPod_force_disable = true;
	    }
	    else if (POD_CONTROLLER_TEMP_WARN_ABOVE > 0 && temp_sensors[FR_CONTROLLER_TEMP-1] > POD_CONTROLLER_TEMP_WARN_ABOVE)
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
	    
	    stat.add("Velocity", FRPod_velocity + " rpm");
	    stat.add("Position", (FRPod_position*180/PI) + "\xc2\xb0");
	    stat.add("Current", FRPod_current + " A");
	    stat.add("Motor Temperature", temp_sensors[FR_MOTOR_TEMP-1] + "\xc2\xb0""C");
	    stat.add("Controller Temperature", temp_sensors[FR_CONTROLLER_TEMP-1] + "\xc2\xb0""C");
	    
	    sensor_data_mutex.unlock();
	}	
	
	void MainBoardDriver::BRPod_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
	{
	    sensor_data_mutex.lock();
	    
	    bool warning;
	    bool error;
	    string msg = "";   
	    
	    //Check over current
	    if (POD_MOTOR_CURRENT_ERROR_ABOVE > 0 && BRPod_current > POD_MOTOR_CURRENT_ERROR_ABOVE)
	    {
	        error = true;
	        msg += "Motor current critical. ";
	        BRPod_force_disable = true;
	    }
	    else if (POD_MOTOR_CURRENT_WARN_ABOVE > 0 && BRPod_current > POD_MOTOR_CURRENT_WARN_ABOVE)
	    {
	        warning = true;
	        msg += "Motor current high. ";	        
	    }
	    
	    //Check motor over temperature
	    if (POD_MOTOR_TEMP_ERROR_ABOVE > 0 && temp_sensors[BR_MOTOR_TEMP-1] > POD_MOTOR_TEMP_ERROR_ABOVE)
	    {
	        error = true;
	        msg += "Motor temperature critical. ";
	        BRPod_force_disable = true;
	    }
	    else if (POD_MOTOR_TEMP_WARN_ABOVE > 0 && temp_sensors[BR_MOTOR_TEMP-1] > POD_MOTOR_TEMP_WARN_ABOVE)
	    {
	        warning = true;
	        msg += "Motor temperature high. ";
	    }
	    
	    //Check controller over temperature
	    if (POD_CONTROLLER_TEMP_ERROR_ABOVE > 0 && temp_sensors[BR_CONTROLLER_TEMP-1] POD_CONTROLLER_TEMP_ERROR_ABOVE)
	    {
	        error = true;
	        msg += "Motor controller temperature critical. ";
	        BRPod_force_disable = true;
	        FRPod_force_disable = true;
	    }
	    else if (POD_CONTROLLER_TEMP_WARN_ABOVE > 0 && temp_sensors[BR_CONTROLLER_TEMP-1] > POD_CONTROLLER_TEMP_WARN_ABOVE)
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
	    stat.add("Velocity", BRPod_velocity + " rpm");
	    stat.add("Position", (BRPod_position*180/PI)  + "\xc2\xb0");
	    stat.add("Current", BRPod_current + " A");
	    stat.add("Motor Temperature", temp_sensors[BR_MOTOR_TEMP-1] + "\xc2\xb0""C");
	    stat.add("Controller Temperature", temp_sensors[BR_CONTROLLER_TEMP-1] + "\xc2\xb0""C");
	    
	    sensor_data_mutex.unlock();
    }
    
	void MainBoardDriver::BLPod_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
	{
	    sensor_data_mutex.lock();
	    
	    bool warning;
	    bool error;
	    string msg = "";   
	    
	    //Check over current
	    if (POD_MOTOR_CURRENT_ERROR_ABOVE > 0 && BLPod_current > POD_MOTOR_CURRENT_ERROR_ABOVE)
	    {
	        error = true;
	        msg += "Motor current critical. ";
	        BLPod_force_disable = true;
	    }
	    else if (POD_MOTOR_CURRENT_WARN_ABOVE > 0 && BLPod_current > POD_MOTOR_CURRENT_WARN_ABOVE)
	    {
	        warning = true;
	        msg += "Motor current high. ";	        
	    }
	    
	    //Check motor over temperature
	    if (POD_MOTOR_TEMP_ERROR_ABOVE > 0 && temp_sensors[BL_MOTOR_TEMP-1] > POD_MOTOR_TEMP_ERROR_ABOVE)
	    {
	        error = true;
	        msg += "Motor temperature critical. ";
	        BLPod_force_disable = true;
	    }
	    else if (POD_MOTOR_TEMP_WARN_ABOVE > 0 && temp_sensors[BL_MOTOR_TEMP-1] > POD_MOTOR_TEMP_WARN_ABOVE)
	    {
	        warning = true;
	        msg += "Motor temperature high. ";
	    }
	    
	    //Check controller over temperature
	    if (POD_CONTROLLER_TEMP_ERROR_ABOVE > 0 && temp_sensors[BL_CONTROLLER_TEMP-1] POD_CONTROLLER_TEMP_ERROR_ABOVE)
	    {
	        error = true;
	        msg += "Motor controller temperature critical. ";
	        BLPod_force_disable = true;
	        FRPod_force_disable = true;
	    }
	    else if (POD_CONTROLLER_TEMP_WARN_ABOVE > 0 && temp_sensors[BL_CONTROLLER_TEMP-1] > POD_CONTROLLER_TEMP_WARN_ABOVE)
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
	    
	    stat.add("Velocity", BLPod_velocity + " rpm");
	    stat.add("Position", (BLPod_position*180/PI) + "\xc2\xb0");
	    stat.add("Current", BLPod_current + " A");
	    stat.add("Motor Temperature", temp_sensors[BL_MOTOR_TEMP-1] + "\xc2\xb0""C");
	    stat.add("Controller Temperature", temp_sensors[BL_CONTROLLER_TEMP-1] + "\xc2\xb0""C");
	    
	    sensor_data_mutex.unlock();
	}
	
	void MainBoardDriver::mainboard_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
	{
	    sensor_data_mutex.lock();
	    
	    control_data_mutex.lock();
	    ros::Duration = ros::Time::now() - last_hs_msg;
	    control_data_mutex.unlock();
	    
        stat.add("Host output status", host_enabled ? "Enabled" : "Disabled");
        stat.add("Board output status", board_enabled ? "Enabled" : "Disabled");
        
        sensor_data_mutex.unlock();
    }
    
	void MainBoardDriver::batt_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat, int index)
	{
	    bool pres = batteries[index].present;
	    
	    if (pres)
	    { 
	        //check low voltage
	        //check low charge
	        //check temp
	    }
	    else
	        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Not Present");
	    
	    
	    stat.add("Present", pres ? "Yes" : "No");
	    stat.add("Charge", pres ? batteries[index].combined_charge + "%" : "No Data");
	    stat.add("Voltage", pres ? batteries[index].combined_voltage + " V" : "No Data");
	    stat.add("Current", pres ? batteries[index].combined_current + " A" : "No Data");
	    stat.add("Average Current", pres ? batteries[index].combined_avg_current + " A" : "No Data");
	    stat.add("Shutdown", pres ? (batteries[index].shutdown ? "Yes" : "No") : "No Data");
	    stat.add("Manufacturer", pres ? batteries[index].mfr : "No Data");
	    stat.add("Device Name", pres ? batteries[index].dev_name : "No Data");
	    stat.add("Battery Type", pres ? batteries[index].chemistry : "No Data");
	    stat.add("Lower Cell Voltage", pres ? batteries[index].lower_voltage + " V" : "No Data");
	    stat.add("Lower Cell Current", pres ? batteries[index].lower_current + " A" : "No Data");
	    stat.add("Lower Cell Average Current", pres ? batteries[index].lower_avg_current + " A" : "No Data");
	    stat.add("Lower Cell Charge", pres ? batteries[index].lower_charge + "%" : "No Data");
	    stat.add("Lower Cell Temperature", pres ? batteries[index].lower_temp + "\xc2\xb0""C" : "No Data");
	    stat.add("Upper Cell Voltage", pres ? batteries[index].upper_voltage + " V" : "No Data");
	    stat.add("Upper Cell Current", pres ? batteries[index].upper_current + " A" : "No Data");
	    stat.add("Upper Cell Average Current", pres ? batteries[index].upper.avg_current + " A" : "No Data");
	    stat.add("Upper Cell Charge", pres ? batteries[index].upper_charge + "%" : "No Data");
	    stat.add("Upper Cell Temperature", pres ? batteries[index].upper_temp + "\xc2\xb0""C" : "No Data");
	}
	
	void MainBoardDriver::power_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
	{
	    bool warning;
	    bool error;
	    string msg = "";   
	    
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
	     if (VICOR_TEMP_ERROR_ABOVE > 0 && temp_sensors[VICOR_TEMP-1] VICOR_TEMP_ERROR_ABOVE)
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
	    stat.add("Main Battery Charge", main_charge + "%)");
	    stat.add("Main Battery Voltage", main_voltage + " V");
	    stat.add("Main Battery Current", main_current + " A");
	    stat.add("Main Batter Average Current", main_avg_current + " A");
	    stat.add("Backup Battery Voltage", backup_voltage + " V");
	    stat.add("Vicor Temperature", temp_sensors[VICOR_TEMP-1] + "\xc2\xb0""C");
	}
	
	void MainBoardDriver::left_drive_temp_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
	{	        
	    if (DRIVE_MOTOR_TEMP_ERROR_ABOVE > 0 && temp_sensors[LEFT_DRIVE_TEMP-1] > DRIVE_MOTOR_TEMP_ERROR_ABOVE)
	        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Temperature critical.");
	    else if (DRIVE_MOTOR_TEMP_WARN_ABOVE > 0 && temp_sensors[LEFT_DRIVE_TEMP-1] > DRIVE_MOTOR_TEMP_WARN_ABOVE)
            stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Temperature high.");
	    else
	        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Temperature OK.");
	        
	    stat.add("Temperature", temp_sensors[LEFT_DRIVE_TEMP-1] + "\xc2\xb0""C");
    }
    
    void MainBoardDriver::right_drive_temp_diagnsotic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        if (DRIVE_MOTOR_TEMP_ERROR_ABOVE > 0 && temp_sensors[RIGHT_DRIVE_TEMP-1] > DRIVE_MOTOR_TEMP_ERROR_ABOVE)
	        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Temperature critical.");
	    else if (DRIVE_MOTOR_TEMP_WARN_ABOVE > 0 && temp_sensors[RIGHT_DRIVE_TEMP-1] > DRIVE_MOTOR_TEMP_WARN_ABOVE)
            stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Temperature high.");
	    else
	        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Temperature OK.");
	        
	    stat.add("Temperature", temp_sensors[RIGHT_DRIVE_TEMP-1] + "\xc2\xb0""C");
    }
    
    void MainBoardDriver::heartbeat_callback(const ros::TimerEvent&)
    {
        warlus_firmware_msgs::MainBoardControl msg;
        msg.type = walrus_firmware_msgs::MainBoardControl::KEEP_ALIVE;
        msg.index = 0;
        msg.value = 0; 
        msg.msg = "";
        to_board.publish(msg);     
    }

}
