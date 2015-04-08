#include "walrus_boomboard_driver/walrus_boomboard_robot.h"
#include <boost/assign/list_of.hpp>

using namespace std;

namespace walrus_boomboard_driver
{

    BoomBoardRobot::BoomBoardRobot(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : WalrusRobotBase(nh, pnh), diagnostic_updater(nh, pnh), last_hs_feedback(0, 0), last_ls_data(0,0), hs_feedback_timeout(0.25), ls_data_timeout(3), led_scale(255)
    {
        //Load parameters
        pnh.param("CO_high_level", CO_HIGH_LEVEL, 2048);
        pnh.param("CNG_high_level", CNG_HIGH_LEVEL, 2048);
        pnh.param("LPG_high_level", LPG_HIGH_LEVEL, 2048);
        pnh.param("H_high_level", H_HIGH_LEVEL, 2048);
        pnh.param("pan_max_position", PAN_POSITION_MAX, 4095);
        pnh.param("pan_min_position", PAN_POSITION_MIN, 0);
        pnh.param("pan_neutral_position", PAN_POSITION_NEUTRAL, 2048);
        pnh.param("pan_position_scale", PAN_POSITION_SCALE, 1.0);
        pnh.param("pan_motor_reverse", PAN_MOTOR_REV, false);
        pnh.param("pan_potentiometer_reverse", PAN_POT_REV, false);
        pnh.param("pan_output_torque_per_amp", PAN_TORQUE_PER_AMP, 1.0);
        pnh.param("tilt_max_position", TILT_POSITION_MAX, 4095);
        pnh.param("tilt_min_position", TILT_POSITION_MIN, 0);
        pnh.param("tilt_neutral_position", TILT_POSITION_NEUTRAL, 2048);
        pnh.param("tilt_position_scale", TILT_POSITION_SCALE, 1.0);
        pnh.param("tilt_motor_reverse", TILT_MOTOR_REV, false);
        pnh.param("tilt_potentiometer_revers", TILT_POT_REV, false);
        pnh.param("tilt_ouput_torque_per_amp", TILT_TORQUE_PER_AMP, 1.0);
        pnh.param("deploy_max_position", DEPLOY_POSITION_MAX, 4096);
        pnh.param("deploy_min_position", DEPLOY_POSITION_MIN, 0);
        pnh.param("deploy_neutral_position", DEPLOY_POSITION_NEUTRAL, 2048);
        pnh.param("deploy_position_scale", DEPLOY_POSITION_SCALE, 1.0);
        pnh.param("deploy_motor_reverse", DEPLOY_MOTOR_REV, false);
        pnh.param("deploy_potentiometer_reverse", DEPLOY_POT_REV, false);
        pnh.param("motor_output_power_scale", MOTOR_OUTPUT_SCALE, 1.0);    
        pnh.param("motor_output_auto_enable", AUTO_ENABLE, false); 
        
        boom_board_connected = false;
        board_enabled = false;
        
        //Setup motor actuator interface
        hardware_interface::ActuatorStateHandle state_handle_deploy("walrus/boom/deploy_joint_actuator", &deploy_position, &deploy_velocity, &deploy_effort);
        asi_.registerHandle(state_handle_deploy);
        hardware_interface::ActuatorStateHandle state_handle_pan("walrus/boom/pan_joint_actuator", &pan_position, &pan_velocity, &pan_effort);
        asi_.registerHandle(state_handle_pan);
        hardware_interface::ActuatorStateHandle state_handle_tilt("walrus/boom/tilt_joint_actuator", &tilt_position, &tilt_velocity, &tilt_effort);
        asi_.registerHandle(state_handle_tilt);
        hardware_interface::ActuatorHandle effort_handle_deploy(state_handle_deploy, &deploy_effort_cmd);
        aei_.registerHandle(effort_handle_deploy);
        hardware_interface::ActuatorHandle effort_handle_pan(state_handle_pan, &pan_effort_cmd);
        aei_.registerHandle(effort_handle_pan);
        hardware_interface::ActuatorHandle effort_handle_tilt(state_handle_tilt, &tilt_effort_cmd);
        aei_.registerHandle(effort_handle_tilt);

        //Add diagnostics updaters
        diagnostic_updater.setHardwareID("Walrus Boom Board");
        diagnostic_updater.add("CO Concentration", this, &BoomBoardRobot::CO_diagnostic_callback);
        diagnostic_updater.add("CNG Concentration", this, &BoomBoardRobot::CNG_diagnostic_callback);
        diagnostic_updater.add("LPG Concentration", this, &BoomBoardRobot::LPG_diagnostic_callback);
        diagnostic_updater.add("Hydrogen Concentration", this, &BoomBoardRobot::H_diagnostic_callback);
        diagnostic_updater.add("Temperature", this, &BoomBoardRobot::temp_diagnostic_callback);
        diagnostic_updater.add("Humidity", this, &BoomBoardRobot::humidity_diagnostic_callback);
        diagnostic_updater.add("Tilt Control", this, &BoomBoardRobot::tilt_diagnostic_callback);
        diagnostic_updater.add("Pan Control", this, &BoomBoardRobot::pan_diagnostic_callback);
        diagnostic_updater.add("Deploy Control", this, &BoomBoardRobot::deploy_diagnostic_callback);
        diagnostic_updater.add("Boom Control Board", this, &BoomBoardRobot::boomboard_diagnostic_callback);
    }

    bool BoomBoardRobot::init()
    {
       //Setup publishers and subscribers to communicate with the embedded board
        hs_control = nh_.advertise<walrus_firmware_msgs::BoomBoardHighSpeedControl>("boom_board/hs_control", 1000);
        to_board = nh_.advertise<walrus_firmware_msgs::BoomBoardControl>("boom_board/PC_to_board_control", 1000);
        heartbeat_timer = nh_.createTimer(ros::Duration(1), &BoomBoardRobot::heartbeat_callback, this);
        cam_led = nh_.subscribe<std_msgs::Float64>("cam_led", 1000, &BoomBoardRobot::led_callback, this);
        set_enable = nh_.subscribe("set_enable", 1000, &BoomBoardRobot::set_enable_callback, this);
        from_board = nh_.subscribe("boom_board/board_to_PC_control", 1000, &BoomBoardRobot::from_board_callback, this);
        ls_data = nh_.subscribe("boom_board/ls_data", 1000, &BoomBoardRobot::ls_data_callback, this);    
        hs_feedback = nh_.subscribe("boom_board/hs_feedback", 1000, &BoomBoardRobot::hs_feedback_callback, this);   
        
      registerInterface(&asi_);
      registerInterface(&aei_);

      std::vector<std::string> actuator_names = boost::assign::list_of
	("walrus/boom/deploy_joint_actuator")
	("walrus/boom/pan_joint_actuator")
	("walrus/boom/tilt_joint_actuator");
      return loadTransmissions(actuator_names);
    }

    void BoomBoardRobot::read(ros::Duration dt)
    {
        boost::lock_guard<boost::mutex> lock(control_data_mutex);
        
        double last_position;
        
        last_position = pan_position;
        pan_position_raw = hs_feedback_msg.pan_position;
        if (PAN_POT_REV)
        {
            pan_limit_max = (pan_position_raw < PAN_POSITION_MIN);
            pan_limit_min = (pan_position_raw > PAN_POSITION_MAX);
        }
        else
        {
            pan_limit_max = (pan_position_raw > PAN_POSITION_MAX);
            pan_limit_min = (pan_position_raw < PAN_POSITION_MIN);           
        }
        pan_position = ((pan_position_raw-PAN_POSITION_NEUTRAL)*PAN_POSITION_SCALE*(PAN_POT_REV ? -1 : 1)); //Convert raw ADC  value (0-4095) to angle       
        pan_velocity = (pan_position - last_position)/dt.toSec();
        pan_current = (hs_feedback_msg.pan_current / 1000.0); //mA -> A
        pan_effort = pan_current * PAN_TORQUE_PER_AMP * (pan_velocity < 0 ? -1 : 1);
        
        last_position = tilt_position;
        tilt_position_raw = hs_feedback_msg.tilt_position;
        if (TILT_POT_REV)
        {
            tilt_limit_max = (tilt_position_raw < TILT_POSITION_MIN);
            tilt_limit_min = (tilt_position_raw > TILT_POSITION_MAX);
        }
        else
        {
            tilt_limit_max = (tilt_position_raw > TILT_POSITION_MAX);
            tilt_limit_min = (tilt_position_raw < TILT_POSITION_MIN);           
        }
        tilt_position = ((tilt_position_raw-TILT_POSITION_NEUTRAL)*TILT_POSITION_SCALE*(TILT_POT_REV ? -1 : 1)); //Convert raw ADC  value (0-4095) to angle       
        tilt_velocity = (tilt_position - last_position)/dt.toSec();
        tilt_current = (hs_feedback_msg.tilt_current / 1000.0); //mA -> A
        tilt_effort = tilt_current * TILT_TORQUE_PER_AMP * (tilt_velocity < 0 ? -1 : 1);
        
        last_position = deploy_position;
        deploy_position_raw = hs_feedback_msg.deploy_position;
        if (DEPLOY_POT_REV)
        {
            deploy_limit_max = (deploy_position_raw < DEPLOY_POSITION_MIN);
            deploy_limit_min = (deploy_position_raw > DEPLOY_POSITION_MAX);
        }
        else
        {
            deploy_limit_max = (deploy_position_raw > DEPLOY_POSITION_MAX);
            deploy_limit_min = (deploy_position_raw < DEPLOY_POSITION_MIN);           
        }
        deploy_position = ((deploy_position_raw-DEPLOY_POSITION_NEUTRAL)*DEPLOY_POSITION_SCALE*(DEPLOY_POT_REV ? -1 : 1)); //Convert raw ADC  value (0-4095) to angle       
        deploy_velocity = (deploy_position - last_position)/dt.toSec();
        deploy_effort = 0;
        
        robot_transmissions_.get<transmission_interface::ActuatorToJointStateInterface>()->propagate();
    }

    void BoomBoardRobot::write(ros::Duration dt)
    {
      robot_transmissions_.get<transmission_interface::JointToActuatorEffortInterface>()->propagate();
        walrus_firmware_msgs::BoomBoardHighSpeedControl hs_control_msg;    
        {
            boost::lock_guard<boost::mutex> lock(control_data_mutex);
            
            //Send motor control message            
            pan_power = pan_effort_cmd;
            if (pan_power < -1) 
                pan_power = -1;
            else if (pan_power > 1) 
                pan_power = 1;
            if ((pan_power > 0 && pan_limit_max) || (pan_power < 0 && pan_limit_min))
                pan_power = 0;
            hs_control_msg.pan_power = (int16_t)(MOTOR_OUTPUT_SCALE*(PAN_MOTOR_REV ? -1 : 1)*pan_power);
            
            tilt_power = tilt_effort_cmd;
            if (tilt_power < -1) 
                tilt_power = -1;
            else if (tilt_power > 1) 
                tilt_power = 1;
            if ((tilt_power > 0 && tilt_limit_max) || (tilt_power < 0 && tilt_limit_min))
                tilt_power = 0;
            hs_control_msg.tilt_power = (int16_t)(MOTOR_OUTPUT_SCALE*(TILT_MOTOR_REV ? -1 : 1)*tilt_power);
            
            deploy_power = deploy_effort_cmd;
            if (deploy_power < -1) 
                deploy_power = -1;
            else if (deploy_power > 1) 
                deploy_power = 1;
            if ((deploy_power > 0 && deploy_limit_max) || (deploy_power < 0 && deploy_limit_min))
                deploy_power = 0;
            hs_control_msg.deploy_power = (int16_t)(MOTOR_OUTPUT_SCALE*(DEPLOY_MOTOR_REV ? -1 : 1)*deploy_power);           
        }
        
        hs_control.publish(hs_control_msg);     
    }

    void BoomBoardRobot::update_diagnostics()
    {
        diagnostic_updater.update();
    }
    
    //Subscription callbacks
    void BoomBoardRobot::led_callback(const std_msgs::Float64::ConstPtr& msg)
    {
        walrus_firmware_msgs::BoomBoardControl led_msg;
        led_msg.type = walrus_firmware_msgs::BoomBoardControl::SET_CAM_LED;
        double intensity = msg->data;
        if (intensity < 0)
            intensity = 0;
        else if (intensity > 1)
            intensity = 1;
        led_msg.value =  (int16_t)(intensity * led_scale);
        led_msg.msg = "";
        to_board.publish(led_msg);
    }
    void BoomBoardRobot::set_enable_callback(const std_msgs::Bool& msg)
    {
        walrus_firmware_msgs::BoomBoardControl enable_msg;
        if (msg.data)
            enable_msg.type = walrus_firmware_msgs::BoomBoardControl::SET_ENABLE;
        else
            enable_msg.type = walrus_firmware_msgs::BoomBoardControl::SET_DISABLE;
        enable_msg.value = 0; 
        enable_msg.msg = "";
        to_board.publish(enable_msg);
    }
    void BoomBoardRobot::hs_feedback_callback(const walrus_firmware_msgs::BoomBoardHighSpeedFeedback& msg)
    {
        boost::lock_guard<boost::mutex> lock(control_data_mutex);
        hs_feedback_msg = msg;
        last_hs_feedback = ros::Time::now();
    }
    void BoomBoardRobot::ls_data_callback(const walrus_firmware_msgs::BoomBoardLowSpeedData& msg)
    {
        boost::lock_guard<boost::mutex> lock(sensor_data_mutex);
        
        temp = msg.temp / 100.0; //hundredths of a deg C -> deg C
        humidity = msg.humidity / 100.0; //hundredths of a % -> %
        CO = msg.CO_sense;
        CNG = msg.CNG_sense;
        LPG = msg.LPG_sense;
        H = msg.H_sense;
        
        last_ls_data = ros::Time::now();
    }
    
    void BoomBoardRobot::from_board_callback(const walrus_firmware_msgs::BoomBoardControl& msg)
    {
        boost::lock_guard<boost::mutex> lock(sensor_data_mutex);
        
        switch (msg.type)
        {
            case walrus_firmware_msgs::BoomBoardControl::STATUS:
                boom_board_status = "Connected, " + msg.msg;
                boom_board_status_level = msg.value;
                if (AUTO_ENABLE && msg.msg.compare("Disabled") == 0)
                {
                    walrus_firmware_msgs::BoomBoardControl status_req_msg;
                    status_req_msg.type = walrus_firmware_msgs::BoomBoardControl::SET_ENABLE;
                    status_req_msg.value = 0; 
                    status_req_msg.msg = "";
                    to_board.publish(status_req_msg);   
                }
            break;
            case walrus_firmware_msgs::BoomBoardControl::ERROR:
                ROS_ERROR_STREAM(msg.msg);
            break;
            default:
                ROS_ERROR("Received invalid control message.");
            break;
        }
    }
    
    
    string BoomBoardRobot::formatDouble(double value, int precision)
    {
        stringstream ss;
        ss << std::fixed << std::setprecision(precision) << value;
        return ss.str();
    }
    
    //Diagnostics updaters
    
    void BoomBoardRobot::CO_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        if (!boom_board_connected)
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No Data");
        else if (CO > CO_HIGH_LEVEL)
            stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "High");
        else
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Safe");
        stat.add("Raw Value", CO);
        stat.add("Threshold", CO_HIGH_LEVEL);
    }
    
    void BoomBoardRobot::CNG_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        if (!boom_board_connected)
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No Data");
        else if (CNG > CNG_HIGH_LEVEL)
            stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "High");
        else
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Safe");
        stat.add("Raw Value", CNG);
        stat.add("Threshold", CNG_HIGH_LEVEL);
    }
    
    void BoomBoardRobot::LPG_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        if (!boom_board_connected)
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No Data");
        else if (LPG > LPG_HIGH_LEVEL)
            stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "High");
        else
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Safe");
        stat.add("Raw Value", LPG);
        stat.add("Threshold", LPG_HIGH_LEVEL);
    }
    
    void BoomBoardRobot::H_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        if (!boom_board_connected)
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No Data");
        else if (H > H_HIGH_LEVEL)
            stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "High");
        else
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Safe");
        stat.add("Raw Value", H);
        stat.add("Threshold", H_HIGH_LEVEL);
    }
    
    void BoomBoardRobot::temp_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        if (!boom_board_connected)
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No Data");
        else
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, formatDouble(temp, 1)+"\xc2\xb0""C");
    }
    
    void BoomBoardRobot::humidity_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        if (!boom_board_connected)
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No Data");
        else
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, formatDouble(humidity, 1)+"%");
    }
    
    void BoomBoardRobot::tilt_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        if (!boom_board_connected)
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No Data");
        else
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
            
        stat.add("Velocity", boom_board_connected ? formatDouble(tilt_velocity, 3) + " rpm" : "No Data");
        stat.add("Position", boom_board_connected ? formatDouble(tilt_position*180/M_PI, 2) + "\xc2\xb0" : "No Data");
        stat.add("Current", boom_board_connected ? formatDouble(tilt_current, 3) + " A" : "No Data");
        stat.add("Raw Position", boom_board_connected ? formatDouble(tilt_position_raw, 0) : "No Data");
        stat.add("Upper Limit Reached", boom_board_connected ? (tilt_limit_max ? "Yes" : "No") : "No Data");
        stat.add("Lower Limit Reached", boom_board_connected ? (tilt_limit_min ? "Yes" : "No") : "No Data");
    }
    
    void BoomBoardRobot::pan_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        if (!boom_board_connected)
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No Data");
        else
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
            
        stat.add("Velocity", boom_board_connected ? formatDouble(pan_velocity, 3) + " rpm" : "No Data");
        stat.add("Position", boom_board_connected ? formatDouble(pan_position*180/M_PI, 2) + "\xc2\xb0" : "No Data");
        stat.add("Current", boom_board_connected ? formatDouble(pan_current, 3) + " A" : "No Data");
        stat.add("Raw Position", boom_board_connected ? formatDouble(pan_position_raw, 0) : "No Data");
        stat.add("Upper Limit Reached", boom_board_connected ? (pan_limit_max ? "Yes" : "No") : "No Data");
        stat.add("Lower Limit Reached", boom_board_connected ? (pan_limit_min ? "Yes" : "No") : "No Data");
    }
    
    void BoomBoardRobot::deploy_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        if (!boom_board_connected)
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No Data");
        else
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
            
        stat.add("Velocity", boom_board_connected ? formatDouble(deploy_velocity, 3) + " rpm" : "No Data");
        stat.add("Position", boom_board_connected ? formatDouble(deploy_position*180/M_PI, 2) + "\xc2\xb0" : "No Data");
        stat.add("Raw Position", boom_board_connected ? formatDouble(deploy_position_raw, 0) : "No Data");
        stat.add("Upper Limit Reached", boom_board_connected ? (deploy_limit_max ? "Yes" : "No") : "No Data");
        stat.add("Lower Limit Reached", boom_board_connected ? (deploy_limit_min ? "Yes" : "No") : "No Data");
    }
    
    void BoomBoardRobot::boomboard_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        boost::lock_guard<boost::mutex> lock(sensor_data_mutex);
        
        bool ls_data_good = false;
        bool hs_feedback_good = false;
        
        if (ls_data_timeout > (ros::Time::now() - last_ls_data)) 
            ls_data_good = true;
        if (hs_feedback_timeout > (ros::Time::now() - last_hs_feedback))
            hs_feedback_good = true;      
        
        if (!boom_board_connected && ls_data_good && hs_feedback_good)
        {
            walrus_firmware_msgs::BoomBoardControl status_req_msg;
            status_req_msg.type = walrus_firmware_msgs::BoomBoardControl::REQ_STATUS;
            status_req_msg.value = 0; 
            status_req_msg.msg = "";
            to_board.publish(status_req_msg);
            boom_board_connected = true;
            boom_board_status_level = diagnostic_msgs::DiagnosticStatus::WARN;
            boom_board_status = "Connected, Waiting for status";
        }
        else if (!ls_data_good || !hs_feedback_good)
        {
            boom_board_connected = false;
            boom_board_status_level = diagnostic_msgs::DiagnosticStatus::ERROR;
            boom_board_status = "Not Connected";
        }
        
        stat.summary(boom_board_status_level, boom_board_status);
        
        stat.add("Pod Motor Feedback", hs_feedback_good ? "OK" : "No Data");
        stat.add("Other Sensor Feedback", ls_data_good ? "OK" : "No Data");
    }
    
    void BoomBoardRobot::heartbeat_callback(const ros::TimerEvent&)
    {
        walrus_firmware_msgs::BoomBoardControl msg;
        msg.type = walrus_firmware_msgs::BoomBoardControl::KEEP_ALIVE;
        msg.value = 0; 
        msg.msg = "";
        to_board.publish(msg);     
    }
    
}
