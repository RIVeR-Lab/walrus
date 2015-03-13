#include "walrus_mainboard_driver/walrus_mainboard_driver.h"

using namespace std;

namespace walrus_mainboard_driver
{
        
    MainBoardDriver::MainBoardDriver(hardware_interface::ActuatorStateInterface& asi,
                  hardware_interface::EffortActuatorInterface &aei,
                  ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : asi_(asi), aei_(aei), diagnostic_updater(nh, pnh), nh(nh), pnh(pnh), last_hs_feedback(0, 0), last_ls_data(0,0), hs_feedback_timeout(0.25), ls_data_timeout(3)
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
        
        pnh.param("pod_auto_enable", POD_AUTO_ENABLE, false);
        pnh.param("frontleft_pod_neutral_position", POD_POSITION_NEUTRAL[FL-1], 0.0);
        pnh.param("frontright_pod_neutral_position", POD_POSITION_NEUTRAL[FR-1], 0.0);
        pnh.param("backright_pod_neutral_position", POD_POSITION_NEUTRAL[BR-1], 0.0);
        pnh.param("backleft_pod_neutral_position", POD_POSITION_NEUTRAL[BL-1], 0.0);
        pnh.param("frontleft_pod_reverse", POD_REV[FL-1], false);
        pnh.param("frontright_pod_reverse", POD_REV[FR-1], false);
        pnh.param("backright_pod_reverse", POD_REV[BR-1], false);
        pnh.param("backleft_pod_reverse", POD_REV[BL-1], false);
        pnh.param("pod_output_torque_per_amp", OUTPUT_TORQUE_PER_AMP, 1.0);
        pnh.param("pod_output_power_neutral", OUTPUT_POWER_NEUTRAL, 1500);
        pnh.param("pod_output_power_limit", OUTPUT_POWER_LIMIT, 500);
        
        pnh.param<string>("water_sense_1_position", WATER_SENSE_POSITION[0], "Position 1");
        pnh.param<string>("water_sense_2_position", WATER_SENSE_POSITION[1], "Position 2");
        pnh.param<string>("water_sense_3_position", WATER_SENSE_POSITION[2], "Position 3");
        pnh.param<string>("water_sense_4_position", WATER_SENSE_POSITION[3], "Position 4");
        pnh.param<string>("water_sense_5_position", WATER_SENSE_POSITION[4], "Position 5");
        pnh.param<string>("water_sense_6_position", WATER_SENSE_POSITION[5], "Position 6");
        
        pnh.param("top_plate_temp_high_above", TOP_PLATE_TEMP_HIGH_ABOVE, -1.0);
        pnh.param("top_plate_temp_critical_above", TOP_PLATE_TEMP_CRITICAL_ABOVE, -1.0);
        pnh.param("abmient_temp_high_above", AMBIENT_TEMP_HIGH_ABOVE, -1.0);
        pnh.param("ambient_temp_critical_above", AMBIENT_TEMP_CRITICAL_ABOVE, -1.0);
        pnh.param("pressure_low_below", PRESSURE_LOW_BELOW, -1.0);
        pnh.param("pressure_critical_below", PRESSURE_CRITICAL_BELOW, -1.0);
        pnh.param("humidity_high_above", HUMIDITY_HIGH_ABOVE, -1.0);
        pnh.param("humidity_critical_above", HUMIDITY_CRITICAL_ABOVE, -1.0);
        
        pnh.param("pod_motor_temp_high_above", POD_MOTOR_TEMP_HIGH_ABOVE, -1.0);
        pnh.param("pod_motor_temp_critical_above", POD_MOTOR_TEMP_CRITICAL_ABOVE, -1.0);
        pnh.param("pod_controller_temp_high_above", POD_CONTROLLER_TEMP_HIGH_ABOVE, -1.0);
        pnh.param("pod_controller_temp_critical_above", POD_CONTROLLER_TEMP_CRITICAL_ABOVE, -1.0);
        pnh.param("pod_motor_current_high_above", POD_MOTOR_CURRENT_HIGH_ABOVE, -1.0);
        pnh.param("pod_motor_current_critical_above", POD_MOTOR_CURRENT_CRITICAL_ABOVE, -1.0);
        
        pnh.param("feedback_timeout", HS_FEEDBACK_TIMEOUT, 250);
        
        pnh.param("main_batt_voltage_low_below", MAIN_BATT_VOLTAGE_LOW_BELOW, -1.0);
        pnh.param("main_batt_charge_low_below", MAIN_BATT_CHARGE_LOW_BELOW, -1.0);
        pnh.param("main_batt_temp_high_above", MAIN_BATT_TEMP_HIGH_ABOVE, -1.0);
        
        pnh.param("vicor_temp_high_above", VICOR_TEMP_HIGH_ABOVE, -1.0);
        pnh.param("vicor_temp_critical_above", VICOR_TEMP_CRITICAL_ABOVE, -1.0);
        pnh.param("backup_batt_voltage_low_below", BACKUP_BATT_VOLTAGE_LOW_BELOW, -1.0);
        pnh.param("main_voltage_low_below", MAIN_VOLTAGE_LOW_BELOW, -1.0);
        pnh.param("main_charge_low_below", MAIN_CHARGE_LOW_BELOW, -1.0);
        
        pnh.param("drive_motor_temp_high_above", DRIVE_MOTOR_TEMP_HIGH_ABOVE, -1.0);
        pnh.param("drive_motor_temp_critical_above", DRIVE_MOTOR_TEMP_CRITICAL_ABOVE, -1.0);
             
        main_board_connected = false;
        board_enabled = false;
        for (int l = 0; l < 4; l++)
        {
            batteries[l].present = false;
            pod_force_disable[l] = false;
        }
        
        //Setup pod actuator interfaces
        hardware_interface::ActuatorStateHandle state_handleFL("walrus/front_left_pod_joint_actuator", &pod_position[FL-1], &pod_velocity[FL-1], &pod_effort[FL-1]);
        asi.registerHandle(state_handleFL);
        hardware_interface::ActuatorStateHandle state_handleFR("walrus/front_right_pod_joint_actuator", &pod_position[FR-1], &pod_velocity[FR-1], &pod_effort[FR-1]);
        asi.registerHandle(state_handleFR);
        hardware_interface::ActuatorStateHandle state_handleBR("walrus/back_right_pod_joint_actuator", &pod_position[BR-1], &pod_velocity[BR-1], &pod_effort[BR-1]);
        asi.registerHandle(state_handleBR);
        hardware_interface::ActuatorStateHandle state_handleBL("walrus/back_left_pod_joint_actuator", &pod_position[BL-1], &pod_velocity[BL-1], &pod_effort[BL-1]);
        asi.registerHandle(state_handleBL);        
        hardware_interface::ActuatorHandle effort_handleFL(state_handleFL, &pod_effort_cmd[FL-1]);
        aei.registerHandle(effort_handleFL);
        hardware_interface::ActuatorHandle effort_handleFR(state_handleFR, &pod_effort_cmd[FR-1]);
        aei.registerHandle(effort_handleFR);
        hardware_interface::ActuatorHandle effort_handleBR(state_handleBR, &pod_effort_cmd[BR-1]);
        aei.registerHandle(effort_handleBR);
        hardware_interface::ActuatorHandle effort_handleBL(state_handleBL, &pod_effort_cmd[BL-1]);
        aei.registerHandle(effort_handleBL);
        
        //Add diagnostics updaters
        diagnostic_updater.setHardwareID("Walrus Main Board");
        diagnostic_updater.add("Top Plate Temperature", this, &MainBoardDriver::tp_temp_diagnostic_callback);
        diagnostic_updater.add("Internal Temperature", this, &MainBoardDriver::int_temp_diagnostic_callback);
        diagnostic_updater.add("Internal Humidity", this, &MainBoardDriver::humidity_diagnostic_callback);
        diagnostic_updater.add("Internal Pressure", this, &MainBoardDriver::pressure_diagnostic_callback);
        for (int l = 0; l < 6; l++)
            diagnostic_updater.add(WATER_SENSE_POSITION[l], boost::bind(&MainBoardDriver::water_diagnostic_callback, this, _1, l));
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
        hs_control = nh.advertise<walrus_firmware_msgs::MainBoardHighSpeedControl>("main_board/hs_control", 1000);
        to_board = nh.advertise<walrus_firmware_msgs::MainBoardControl>("main_board/PC_to_board_control", 1000);
        heartbeat_timer = nh.createTimer(ros::Duration(1), &MainBoardDriver::heartbeat_callback, this);
        front_led = nh.subscribe<std_msgs::Float64>("front_led", 1000, boost::bind(&MainBoardDriver::led_callback, this, _1, FRONT_CAM_LED-1));
        back_led = nh.subscribe<std_msgs::Float64>("back_led", 1000, boost::bind(&MainBoardDriver::led_callback, this, _1, BACK_CAM_LED-1));
        bottom_led = nh.subscribe<std_msgs::Float64>("bottom_led", 1000, boost::bind(&MainBoardDriver::led_callback, this, _1, BOTTOM_CAM_LED-1));
        set_enable = nh.subscribe("set_enable", 1000, &MainBoardDriver::set_enable_callback, this);
        from_board = nh.subscribe("main_board/board_to_PC_control", 1000, &MainBoardDriver::from_board_callback, this);
        ls_data = nh.subscribe("main_board/ls_data", 1000, &MainBoardDriver::ls_data_callback, this);    
        hs_feedback = nh.subscribe("main_board/hs_feedback", 1000, &MainBoardDriver::hs_feedback_callback, this);   
        return true;
    }
    
    void MainBoardDriver::read(ros::Duration dt)
    {       
        boost::lock_guard<boost::mutex> lock(control_data_mutex);
       
        for (int l = 0; l < 4; l++)
        {
            double position, delta;
            position = (hs_feedback_msg.pod_position[l]/1023.0)*2*M_PI; //Convert raw ADC value (0-1023) to angle (0-2pi)
            if (POD_REV[l])
                position = (2*M_PI) - position;
            position -= POD_POSITION_NEUTRAL[l];
            if (position < 0)
                position += 2*M_PI;
            delta = position - pod_position[l];
            if (delta > M_PI) //Decrease angle across 0
                pod_velocity[l] = (delta-2*M_PI)/dt.toSec();
            else if (delta < -M_PI) //Increase angle across 0
                pod_velocity[l] = (delta+2*M_PI)/dt.toSec();
            else  
                pod_velocity[l] = delta/dt.toSec();
            pod_position[l] = position;        
            pod_current[l] = (hs_feedback_msg.motor_current[l] / 1000.0); //mA -> A
            pod_effort[l] = pod_effort[l] * OUTPUT_TORQUE_PER_AMP;
        }        
    }
    
    void MainBoardDriver::write(ros::Duration dt)
    {            
        walrus_firmware_msgs::MainBoardHighSpeedControl hs_control_msg;    
        {
            boost::lock_guard<boost::mutex> lock(control_data_mutex);
            //Send motor control message            
            for (int l = 0; l < 4; l++)
            {
                if (pod_force_disable[l])
                    pod_power[l] = OUTPUT_POWER_NEUTRAL;
                else
                    pod_power[l] = (pod_effort_cmd[l]*OUTPUT_POWER_LIMIT) + OUTPUT_POWER_NEUTRAL;
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
    void MainBoardDriver::led_callback(const std_msgs::Float64::ConstPtr& msg, int index)
    {
        walrus_firmware_msgs::MainBoardControl led_msg;
        led_msg.type = walrus_firmware_msgs::MainBoardControl::REQ_BATT_INFO;
        led_msg.index = index;
        double intensity = msg->data;
        if (intensity < 0)
            intensity = 0;
        else if (intensity > 1)
            intensity = 1;
        led_msg.value =  (int16_t)(intensity * 255);
        led_msg.msg = "";
        to_board.publish(led_msg);
    }
    void MainBoardDriver::set_enable_callback(const std_msgs::Bool& msg)
    {
        walrus_firmware_msgs::MainBoardControl enable_msg;
        if (msg.data)
            enable_msg.type = walrus_firmware_msgs::MainBoardControl::SET_ENABLE;
        else
            enable_msg.type = walrus_firmware_msgs::MainBoardControl::SET_DISABLE;
        enable_msg.index = 0;
        enable_msg.value = 0; 
        enable_msg.msg = "";
        to_board.publish(enable_msg);
    }
    void MainBoardDriver::hs_feedback_callback(const walrus_firmware_msgs::MainBoardHighSpeedFeedback& msg)
    {
        boost::lock_guard<boost::mutex> lock(control_data_mutex);
        hs_feedback_msg = msg;
        last_hs_feedback = ros::Time::now();
    }
    void MainBoardDriver::ls_data_callback(const walrus_firmware_msgs::MainBoardLowSpeedData& msg)
    {
        boost::lock_guard<boost::mutex> lock(sensor_data_mutex);
        
        //Temp sensors
        for (int l = 0; l < 10; l++)
            temp_sensors[l] = msg.temp_sense[l] / 10.0; //tenths of a deg C -> deg C
        
        //Ambient temp
        ambient_temp = msg.board_temp / 100.0; //hundredths of a deg C -> deg C
        
        //Humidity
        humidity = msg.humidity / 100.0; //hundredths of a % -> %
        
        //Pressure
        pressure = msg.pressure / 1000.0; //Pa -> kPa
        
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
            if (msg.batt_present & (1 << l))
            {
                //if it wasn't present, ask it for mfr and name data
                if (!batteries[l].present)
                {
                    batteries[l].mfr = "Not available";
                    batteries[l].dev_name = "Not available";
                    batteries[l].chemistry = "Not available";
                    walrus_firmware_msgs::MainBoardControl info_req_msg;
                    info_req_msg.type = walrus_firmware_msgs::MainBoardControl::REQ_BATT_INFO;
                    info_req_msg.index = l;
                    info_req_msg.value = 0; 
                    info_req_msg.msg = "";
                    to_board.publish(info_req_msg);
                }
                batteries[l].present = true;  
                batteries[l].lower_voltage = msg.lcell_voltage[l] / 1000.0; //mV -> V
                batteries[l].lower_current = msg.lcell_current[l] / 1000.0; //mA -> A
                batteries[l].lower_charge = msg.lcell_charge[l];
                batteries[l].lower_temp = msg.lcell_temp[l] / 100.0; //hunderedths of a deg C -> deg C
                batteries[l].lower_avg_current = msg.lcell_avgcurr[l] / 1000.0; //mA -> A
                batteries[l].upper_voltage = msg.ucell_voltage[l] / 1000.0; //mV -> V
                batteries[l].upper_current = msg.ucell_current[l] / 1000.0; //mA -> A
                batteries[l].upper_charge = msg.ucell_charge[l];
                batteries[l].upper_temp = msg.ucell_temp[l] / 100.0; //hunderedths of a deg C -> deg C
                batteries[l].upper_avg_current = msg.ucell_avgcurr[l] / 1000.0; //mA -> A
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
        
        last_ls_data = ros::Time::now();
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
                main_board_status = "Connected, " + msg.msg;
                main_board_status_level = msg.value;
                if (POD_AUTO_ENABLE && msg.msg == "Disabled")
                {
                    walrus_firmware_msgs::MainBoardControl status_req_msg;
                    status_req_msg.type = walrus_firmware_msgs::MainBoardControl::REQ_STATUS;
                    status_req_msg.index = 0;
                    status_req_msg.value = 0; 
                    status_req_msg.msg = "";
                    to_board.publish(status_req_msg);
                }
            break;
            case walrus_firmware_msgs::MainBoardControl::ERROR:
                ROS_ERROR_STREAM(msg.msg);
            break;
            default:
                ROS_ERROR("Received invalid control message.");
            break;
        }
    }
    
    
    string MainBoardDriver::formatDouble(double value, int precision)
    {
        stringstream ss;
        ss << std::fixed << std::setprecision(precision) << value;
        return ss.str();
    }
    
    //Diagnostics updaters
    
    void MainBoardDriver::tp_temp_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
         if (!main_board_connected)
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No Data");
        else if (TOP_PLATE_TEMP_CRITICAL_ABOVE > 0 && temp_sensors[TOP_PLATE_TEMP-1] > TOP_PLATE_TEMP_CRITICAL_ABOVE)
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Top plate temperature critical");
        else if (TOP_PLATE_TEMP_HIGH_ABOVE > 0 && temp_sensors[TOP_PLATE_TEMP-1] > TOP_PLATE_TEMP_HIGH_ABOVE)
            stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Top plate temperature high");
        else
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Top plate temperature OK");
            
        stat.add("Top Plate Temperature", main_board_connected ? formatDouble(temp_sensors[TOP_PLATE_TEMP-1],1) + "\xc2\xb0""C" : "No Data");
    }
    
    void MainBoardDriver::int_temp_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        if (!main_board_connected)
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No Data");
        else if (AMBIENT_TEMP_CRITICAL_ABOVE > 0 && ambient_temp > AMBIENT_TEMP_CRITICAL_ABOVE)
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Internal temperature critical");
        else if (AMBIENT_TEMP_HIGH_ABOVE > 0 && ambient_temp > AMBIENT_TEMP_HIGH_ABOVE)
            stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Internal temperature high");
        else
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Internal temperature OK");   
            
        stat.add("Internal Temperature", main_board_connected ? formatDouble(ambient_temp,1) + "\xc2\xb0""C" : "No Data");
    }
    
    void MainBoardDriver::humidity_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        if (!main_board_connected)
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No Data");
        else if (HUMIDITY_CRITICAL_ABOVE > 0 && humidity > HUMIDITY_CRITICAL_ABOVE)
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Internal humidity critical");
        else if (HUMIDITY_HIGH_ABOVE > 0 && humidity > HUMIDITY_HIGH_ABOVE)
            stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Internal humidity high");
        else
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Internal humidity OK");   
    
        stat.add("Internal Humidity", main_board_connected ? formatDouble(humidity,1) + "%" : "No Data");
    }
    
    void MainBoardDriver::pressure_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        if (!main_board_connected)
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No Data");
        else if (PRESSURE_CRITICAL_BELOW> 0 && ambient_temp < PRESSURE_CRITICAL_BELOW)
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Internal pressure critical");
        else if (PRESSURE_LOW_BELOW > 0 && ambient_temp < PRESSURE_LOW_BELOW)
            stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Internal pressure low");
        else
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Internal pressure OK");   
        stat.add("Internal Pressure", main_board_connected ? formatDouble(pressure,3) + " kPa" : "No Data");
    }
    
    void MainBoardDriver::water_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat, int index)
    {
        if (!main_board_connected)
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No Data");
        else if (water[index])
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "No Water Detected");
        else
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Leak Detected");
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
            if (POD_MOTOR_CURRENT_CRITICAL_ABOVE > 0 && pod_current[index] > POD_MOTOR_CURRENT_CRITICAL_ABOVE)
            {
                error = true;
                msg += "Motor current critical. ";
                pod_force_disable[index] = true;
            }
            else if (POD_MOTOR_CURRENT_HIGH_ABOVE > 0 && pod_current[index] > POD_MOTOR_CURRENT_HIGH_ABOVE)
            {
                warning = true;
                msg += "Motor current high. ";            
            }
            
            //Check motor over temperature
            if (POD_MOTOR_TEMP_CRITICAL_ABOVE > 0 && temp_sensors[MOTOR_TEMP[index]-1] > POD_MOTOR_TEMP_CRITICAL_ABOVE)
            {
                error = true;
                msg += "Motor temperature critical, motor disabled.";
                pod_force_disable[index] = true;
            }
            else if (POD_MOTOR_TEMP_HIGH_ABOVE > 0 && temp_sensors[MOTOR_TEMP[index]-1] > POD_MOTOR_TEMP_HIGH_ABOVE)
            {
                warning = true;
                msg += "Motor temperature high. ";
            }
            else
                pod_force_disable[index] = false;
            
            //Check controller over temperature
            if (POD_CONTROLLER_TEMP_CRITICAL_ABOVE > 0 && temp_sensors[CONTROLLER_TEMP[index]-1] > POD_CONTROLLER_TEMP_CRITICAL_ABOVE)
            {
                error = true;
                msg += "Motor controller temperature critical. motor disabled.";
                pod_force_disable[index] = true;
            }
            else if (POD_CONTROLLER_TEMP_HIGH_ABOVE > 0 && temp_sensors[CONTROLLER_TEMP[index]-1] > POD_CONTROLLER_TEMP_HIGH_ABOVE)
            {
                warning = true;
                msg += "Motor controller temperature high. ";
            }
            else
                pod_force_disable[index] = false;
            
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
        
        stat.add("Velocity", main_board_connected ? formatDouble(pod_velocity[index], 0) + " rpm" : "No Data");
        stat.add("Position", main_board_connected ? formatDouble((pod_position[index]*180/M_PI), 2) + "\xc2\xb0" : "No Data");
        stat.add("Current", main_board_connected ? formatDouble(pod_current[index], 3) + " A" : "No Data");
        stat.add("Motor Temperature", main_board_connected ? formatDouble(temp_sensors[MOTOR_TEMP[index]-1], 1) + "\xc2\xb0""C" : "No Data");
        stat.add("Controller Temperature", main_board_connected ? formatDouble(temp_sensors[CONTROLLER_TEMP[index]-1], 1) + "\xc2\xb0""C" : "No Data");
    }
    
    
    void MainBoardDriver::mainboard_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        boost::lock_guard<boost::mutex> lock(sensor_data_mutex);
        
        bool ls_data_good = false;
        bool hs_feedback_good = false;
        
        if (ls_data_timeout < (ros::Time::now() - last_ls_data)) 
            ls_data_good = true;
        if (hs_feedback_timeout < (ros::Time::now() - last_hs_feedback))
            hs_feedback_good = true;      
        
        if (!main_board_connected && ls_data_good && hs_feedback_good)
        {
            walrus_firmware_msgs::MainBoardControl status_req_msg;
            status_req_msg.type = walrus_firmware_msgs::MainBoardControl::REQ_STATUS;
            status_req_msg.index = 0;
            status_req_msg.value = 0; 
            status_req_msg.msg = "";
            to_board.publish(status_req_msg);
            main_board_connected = true;
            main_board_status_level = diagnostic_msgs::DiagnosticStatus::WARN;
            main_board_status = "Connected, Waiting for status";
        }
        else if (!ls_data_good || !hs_feedback_good)
        {
            main_board_connected = false;
            main_board_status_level = diagnostic_msgs::DiagnosticStatus::ERROR;
            main_board_status = "Not Connected";
        }
        
        stat.summary(main_board_status_level, main_board_status);
        
        stat.add("Pod Motor Feedback", hs_feedback_good ? "OK" : "No Data");
        stat.add("Other Sensor Feedback", ls_data_good ? "OK" : "No Data");
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
                if (MAIN_BATT_VOLTAGE_LOW_BELOW > 0 && batteries[index].combined_voltage < MAIN_BATT_VOLTAGE_LOW_BELOW)
                {
                    warning = true;
                    msg += "Voltage low. ";
                }
                //check charge
                if (MAIN_BATT_CHARGE_LOW_BELOW >   0 && batteries[index].combined_charge < MAIN_BATT_CHARGE_LOW_BELOW)
                {
                    warning = true;
                    msg += "Charge low. ";
                }
                //check temp of high cell
                if (MAIN_BATT_TEMP_HIGH_ABOVE > 0 && batteries[index].upper_temp > MAIN_BATT_TEMP_HIGH_ABOVE)
                {
                    warning = true;
                    msg += "Upper cell temperature high. ";
                }
                //check temp of low cell
                if (MAIN_BATT_TEMP_HIGH_ABOVE > 0 && batteries[index].lower_temp > MAIN_BATT_TEMP_HIGH_ABOVE)
                {
                    warning = true;
                    msg += "Lower cell temperature high. ";
                }
                
                uint8_t level = diagnostic_msgs::DiagnosticStatus::OK;
                if (warning)
                    level = diagnostic_msgs::DiagnosticStatus::WARN;
                else
                    msg = "Everything OK";
                
                stat.summary(level, msg);
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
        stat.add("Charge", pres ? formatDouble(batteries[index].combined_charge, 0) + "%" : "No Data");
        stat.add("Voltage", pres ? formatDouble(batteries[index].combined_voltage, 3) + " V" : "No Data");
        stat.add("Current", pres ? formatDouble(batteries[index].combined_current, 3) + " A" : "No Data");
        stat.add("Average Current", pres ? formatDouble(batteries[index].combined_avg_current, 3) + " A" : "No Data");
        stat.add("Shutdown", pres ? (batteries[index].shutdown ? "Yes" : "No") : "No Data");
        stat.add("Manufacturer", pres ? batteries[index].mfr : "No Data");
        stat.add("Device Name", pres ? batteries[index].dev_name : "No Data");
        stat.add("Battery Type", pres ? batteries[index].chemistry : "No Data");
        stat.add("Lower Cell Voltage", pres ? formatDouble(batteries[index].lower_voltage, 3) + " V" : "No Data");
        stat.add("Lower Cell Current", pres ? formatDouble(batteries[index].lower_current, 3) + " A" : "No Data");
        stat.add("Lower Cell Average Current", pres ? formatDouble(batteries[index].lower_avg_current, 3) + " A" : "No Data");
        stat.add("Lower Cell Charge", pres ? formatDouble(batteries[index].lower_charge, 0) + "%" : "No Data");
        stat.add("Lower Cell Temperature", pres ? formatDouble(batteries[index].lower_temp, 1) + "\xc2\xb0""C" : "No Data");
        stat.add("Upper Cell Voltage", pres ? formatDouble(batteries[index].upper_voltage, 3) + " V" : "No Data");
        stat.add("Upper Cell Current", pres ? formatDouble(batteries[index].upper_current, 3) + " A" : "No Data");
        stat.add("Upper Cell Average Current", pres ? formatDouble(batteries[index].upper_avg_current, 3) + " A" : "No Data");
        stat.add("Upper Cell Charge", pres ? formatDouble(batteries[index].upper_charge, 0) + "%" : "No Data");
        stat.add("Upper Cell Temperature", pres ? formatDouble(batteries[index].upper_temp, 1) + "\xc2\xb0""C" : "No Data");
    }
    
    void MainBoardDriver::power_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        bool warning;
        bool error;
        string msg = "";   
        
        if (main_board_connected)
        {    
            //Check charge
            if (MAIN_CHARGE_LOW_BELOW > 0 && main_charge < MAIN_CHARGE_LOW_BELOW)
            {
                warning = true;
                msg += "Main battery low. ";
            }
            
            //Check main voltage
            if (MAIN_VOLTAGE_LOW_BELOW > 0 && main_voltage < MAIN_VOLTAGE_LOW_BELOW)
            {
                warning = true;
                msg += "Main battery voltage low. ";
            }
            
            //Check backup voltage
            if (BACKUP_BATT_VOLTAGE_LOW_BELOW > 0 && backup_voltage < BACKUP_BATT_VOLTAGE_LOW_BELOW)
            {
                warning = true;
                if (backup_voltage < 1)
                    msg += "Backup battery not present. ";
                else
                    msg += "Backup battery voltage low. ";
            }
            
            //Check vicor temperature
             if (VICOR_TEMP_CRITICAL_ABOVE > 0 && temp_sensors[VICOR_TEMP-1] > VICOR_TEMP_CRITICAL_ABOVE)
            {
                error = true;
                msg += "Vicor temperature critical. ";
            }
            else if (VICOR_TEMP_HIGH_ABOVE > 0 && temp_sensors[VICOR_TEMP-1] > VICOR_TEMP_HIGH_ABOVE)
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
        
        
        stat.add("Main Battery Charge", main_board_connected ? formatDouble(main_charge, 0) + "%" : "No Data");
        stat.add("Main Battery Voltage", main_board_connected ? formatDouble(main_voltage, 3) + " V" : "No Data");
        stat.add("Main Battery Current", main_board_connected ? formatDouble(main_current, 3) + " A" : "No Data");
        stat.add("Main Batter Average Current", main_board_connected ? formatDouble(main_avg_current, 3) + " A" : "No Data");
        stat.add("Backup Battery Voltage", main_board_connected ? formatDouble(backup_voltage, 3) + " V" : "No Data");
        stat.add("Vicor Temperature", main_board_connected ? formatDouble(temp_sensors[VICOR_TEMP-1], 1) + "\xc2\xb0""C" : "No Data");
    }
    
    void MainBoardDriver::left_drive_temp_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {        
        if (main_board_connected)
        {    
            if (DRIVE_MOTOR_TEMP_CRITICAL_ABOVE > 0 && temp_sensors[LEFT_DRIVE_TEMP-1] > DRIVE_MOTOR_TEMP_CRITICAL_ABOVE)
                stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Temperature critical.");
            else if (DRIVE_MOTOR_TEMP_HIGH_ABOVE > 0 && temp_sensors[LEFT_DRIVE_TEMP-1] > DRIVE_MOTOR_TEMP_HIGH_ABOVE)
                stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Temperature high.");
            else
                stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Temperature OK.");
        }
        else
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No Data");
            
        stat.add("Temperature", main_board_connected ? formatDouble(temp_sensors[LEFT_DRIVE_TEMP-1], 1) + "\xc2\xb0""C" : "No Data");
    }
    
    void MainBoardDriver::right_drive_temp_diagnsotic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        if (main_board_connected)
        {
            if (DRIVE_MOTOR_TEMP_CRITICAL_ABOVE > 0 && temp_sensors[RIGHT_DRIVE_TEMP-1] > DRIVE_MOTOR_TEMP_CRITICAL_ABOVE)
                stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Temperature critical.");
            else if (DRIVE_MOTOR_TEMP_HIGH_ABOVE > 0 && temp_sensors[RIGHT_DRIVE_TEMP-1] > DRIVE_MOTOR_TEMP_HIGH_ABOVE)
                stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Temperature high.");
            else
                stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Temperature OK.");
        }
        else
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No Data");
            
        stat.add("Temperature", main_board_connected ? formatDouble(temp_sensors[RIGHT_DRIVE_TEMP-1], 1) + "\xc2\xb0""C" : "No Data");
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
