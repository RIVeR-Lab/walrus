#include "walrus_mainboard_driver/walrus_mainboard_robot.h"
#include <boost/assign/list_of.hpp>

using namespace std;

namespace walrus_mainboard_driver
{

    MainBoardRobot::MainBoardRobot(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), diagnostic_updater(nh, pnh), last_sensor_data(0,0), sensor_data_timeout(3), led_scale(255)
    {
        //Load parameters    
        pnh.param("frontleft_motor_temp_id", FL_POD_MOTOR_TEMP, 1);   
        pnh.param("frontright_motor_temp_id", FR_POD_MOTOR_TEMP, 2);
        pnh.param("backright_motor_temp_id", BR_POD_MOTOR_TEMP, 3);
        pnh.param("backleft_motor_temp_id", BL_POD_MOTOR_TEMP, 4);
        pnh.param("victor_temp_id", VICOR_TEMP, 7);
        pnh.param("top_plate_temp_id", TOP_PLATE_TEMP, 8);
        pnh.param("right_drive_motor_temp_id", RIGHT_DRIVE_TEMP, 9);
        pnh.param("left_drive_motor_temp_id", LEFT_DRIVE_TEMP, 10);
        pnh.param("front_cam_led_id", FRONT_CAM_LED, 1);
        pnh.param("bottom_cam_led_id", BOTTOM_CAM_LED, 2);
        pnh.param("back_cam_led_id", BACK_CAM_LED, 3);
        pnh.param("backup_batt_voltage_id", BACKUP_BATT_VOLTAGE, 1);
        
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

        pnh.param("main_batt_voltage_low_below", MAIN_BATT_VOLTAGE_LOW_BELOW, -1.0);
        pnh.param("main_batt_charge_low_below", MAIN_BATT_CHARGE_LOW_BELOW, -1.0);
        pnh.param("main_batt_temp_high_above", MAIN_BATT_TEMP_HIGH_ABOVE, -1.0);
        pnh.param("backup_batt_volts_per_count", BACKUP_BATT_VOLTS_PER_COUNT, 0.001);
        
        pnh.param("vicor_temp_high_above", VICOR_TEMP_HIGH_ABOVE, -1.0);
        pnh.param("vicor_temp_critical_above", VICOR_TEMP_CRITICAL_ABOVE, -1.0);
        pnh.param("backup_batt_voltage_low_below", BACKUP_BATT_VOLTAGE_LOW_BELOW, -1.0);
        pnh.param("main_voltage_low_below", MAIN_VOLTAGE_LOW_BELOW, -1.0);
        pnh.param("main_charge_low_below", MAIN_CHARGE_LOW_BELOW, -1.0);
        
        pnh.param("drive_motor_temp_high_above", DRIVE_MOTOR_TEMP_HIGH_ABOVE, -1.0);
        pnh.param("drive_motor_temp_critical_above", DRIVE_MOTOR_TEMP_CRITICAL_ABOVE, -1.0);
             
        main_board_connected = false;
        for (int l = 0; l < 4; l++)
            batteries[l].present = false;

        
        //Add diagnostics updaters
        diagnostic_updater.setHardwareID("Walrus Main Board");
        diagnostic_updater.add("Top Plate Temperature", this, &MainBoardRobot::tp_temp_diagnostic_callback);
        diagnostic_updater.add("Internal Temperature", this, &MainBoardRobot::int_temp_diagnostic_callback);
        diagnostic_updater.add("Internal Humidity", this, &MainBoardRobot::humidity_diagnostic_callback);
        diagnostic_updater.add("Internal Pressure", this, &MainBoardRobot::pressure_diagnostic_callback);
        for (int l = 0; l < 6; l++)
            diagnostic_updater.add(WATER_SENSE_POSITION[l], boost::bind(&MainBoardRobot::water_diagnostic_callback, this, _1, l));
        diagnostic_updater.add("Front Left Pod Temperature", boost::bind(&MainBoardRobot::pod_temp_diagnostic_callback, this, _1, FL_POD_MOTOR_TEMP));
        diagnostic_updater.add("Front Right Pod Temperature", boost::bind(&MainBoardRobot::pod_temp_diagnostic_callback, this, _1, FR_POD_MOTOR_TEMP));
        diagnostic_updater.add("Back Right Pod Temperature", boost::bind(&MainBoardRobot::pod_temp_diagnostic_callback, this, _1, BR_POD_MOTOR_TEMP));
        diagnostic_updater.add("Back Left Pod Temperature", boost::bind(&MainBoardRobot::pod_temp_diagnostic_callback, this, _1, BL_POD_MOTOR_TEMP));
        diagnostic_updater.add("Main Control Board", this, &MainBoardRobot::mainboard_diagnostic_callback);
        diagnostic_updater.add("Battery 1", boost::bind(&MainBoardRobot::batt_diagnostic_callback, this, _1, 0));
        diagnostic_updater.add("Battery 2", boost::bind(&MainBoardRobot::batt_diagnostic_callback, this, _1, 1));
        diagnostic_updater.add("Battery 3", boost::bind(&MainBoardRobot::batt_diagnostic_callback, this, _1, 2));
        diagnostic_updater.add("Battery 4", boost::bind(&MainBoardRobot::batt_diagnostic_callback, this, _1, 3));
        diagnostic_updater.add("Power Systems", this, &MainBoardRobot::power_diagnostic_callback);
        diagnostic_updater.add("Left Drive Motor Temperature", this, &MainBoardRobot::left_drive_temp_diagnostic_callback);
        diagnostic_updater.add("Right Drive Motor Temperature", this, &MainBoardRobot::right_drive_temp_diagnsotic_callback);
        
        //Setup publishers and subscribers to communicate with the embedded board
        to_board = nh_.advertise<walrus_firmware_msgs::MainBoardControl>("main_board/PC_to_board_control", 1000);
        heartbeat_timer = nh_.createTimer(ros::Duration(1), &MainBoardRobot::heartbeat_callback, this);
        front_led = nh_.subscribe<std_msgs::Float64>("front_led", 1000, boost::bind(&MainBoardRobot::led_callback, this, _1, FRONT_CAM_LED-1));
        back_led = nh_.subscribe<std_msgs::Float64>("back_led", 1000, boost::bind(&MainBoardRobot::led_callback, this, _1, BACK_CAM_LED-1));
        bottom_led = nh_.subscribe<std_msgs::Float64>("bottom_led", 1000, boost::bind(&MainBoardRobot::led_callback, this, _1, BOTTOM_CAM_LED-1));
        from_board = nh_.subscribe("main_board/board_to_PC_control", 1000, &MainBoardRobot::from_board_callback, this);
        sensor_data = nh_.subscribe("main_board/sensor_data", 1000, &MainBoardRobot::sensor_data_callback, this);    
    }
    
    void MainBoardRobot::update_diagnostics()
    {
        diagnostic_updater.update();
    } 
    
    //Subscription callbacks
    void MainBoardRobot::led_callback(const std_msgs::Float64::ConstPtr& msg, int index)
    {
        walrus_firmware_msgs::MainBoardControl led_msg;
        led_msg.type = walrus_firmware_msgs::MainBoardControl::SET_LED;
        led_msg.index = index;
        double intensity = msg->data;
        if (intensity < 0)
            intensity = 0;
        else if (intensity > 1)
            intensity = 1;
        led_msg.value =  (int16_t)(intensity * led_scale);
        led_msg.msg = "";
        to_board.publish(led_msg);
    }
    void MainBoardRobot::sensor_data_callback(const walrus_firmware_msgs::MainBoardSensorData& msg)
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
        backup_voltage = msg.tension[BACKUP_BATT_VOLTAGE-1]*BACKUP_BATT_VOLTS_PER_COUNT;
        
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
        main_charge /= count;
        
        last_sensor_data = ros::Time::now();
    }
    
    void MainBoardRobot::from_board_callback(const walrus_firmware_msgs::MainBoardControl& msg)
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
            case walrus_firmware_msgs::MainBoardControl::ERROR:
                ROS_ERROR_STREAM(msg.msg);
            break;
            default:
                ROS_ERROR("Received invalid control message.");
            break;
        }
    }
    
    
    string MainBoardRobot::formatDouble(double value, int precision)
    {
        stringstream ss;
        ss << std::fixed << std::setprecision(precision) << value;
        return ss.str();
    }
    
    //Diagnostics updaters
    
    void MainBoardRobot::tp_temp_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
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
    
    void MainBoardRobot::int_temp_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
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
    
    void MainBoardRobot::humidity_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
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
    
    void MainBoardRobot::pressure_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
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
    
    void MainBoardRobot::water_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat, int index)
    {
        if (!main_board_connected)
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No Data");
        else if (water[index])
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "No Water Detected");
        else
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Leak Detected");
    }
    
    void MainBoardRobot::pod_temp_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat, int index)
    {
        uint8_t level = diagnostic_msgs::DiagnosticStatus::OK;
        if (!main_board_connected)
            level = diagnostic_msgs::DiagnosticStatus::ERROR;
        else if (POD_MOTOR_TEMP_HIGH_ABOVE > 0 && temp_sensors[index] > POD_MOTOR_TEMP_HIGH_ABOVE)
            level = diagnostic_msgs::DiagnosticStatus::WARN;    
        stat.summary(level, main_board_connected ? formatDouble(temp_sensors[index], 1) + "\xc2\xb0""C" : "No Data");
    }
    
    void MainBoardRobot::mainboard_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        boost::lock_guard<boost::mutex> lock(sensor_data_mutex);
        
        main_board_connected = sensor_data_timeout > (ros::Time::now() - last_sensor_data);

        if (main_board_connected)
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Connected");
        else
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Not Connected");
    }
    
    void MainBoardRobot::batt_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat, int index)
    {
        bool pres = batteries[index].present;
        bool warning = false;
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
    
    void MainBoardRobot::power_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        bool warning = false;
        bool error = false;
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
        stat.add("Main Battery Average Current", main_board_connected ? formatDouble(main_avg_current, 3) + " A" : "No Data");
        stat.add("Backup Battery Voltage", main_board_connected ? formatDouble(backup_voltage, 3) + " V" : "No Data");
        stat.add("Vicor Temperature", main_board_connected ? formatDouble(temp_sensors[VICOR_TEMP-1], 1) + "\xc2\xb0""C" : "No Data");
    }
    
    void MainBoardRobot::left_drive_temp_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
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
    
    void MainBoardRobot::right_drive_temp_diagnsotic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
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
    
    void MainBoardRobot::heartbeat_callback(const ros::TimerEvent&)
    {
        walrus_firmware_msgs::MainBoardControl msg;
        msg.type = walrus_firmware_msgs::MainBoardControl::KEEP_ALIVE;
        msg.index = 0;
        msg.value = 0; 
        msg.msg = "";
        to_board.publish(msg);     
    }

}
