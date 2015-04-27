#include "walrus_pod_hw/walrus_pod_hw.h"

namespace walrus_pod_hw
{
    WalrusPodHW::WalrusPodHW(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::string front_controller_device, std_string back_controller_device) 
    : WalrusRobotBase(nh, pnh), diagnostic_updater(nh, pnh), FL_POD(0), BL_POD(1), FR_POD(2), BL_POD(3), CONTROLLER_MASK(1), FRONT_CONTROLLER(0), BACK_CONTROLLER(1), pm_feedback_timeout(0.25), last_pm_feedback(0,0)
    {    
        controllers[0] = new RoboteqMotorController(1000, 1000, 0, 0);
        controllers[0] = new RoboteqMotorController(1000, 1000, 0, 0);
        
        controller_devices[FRONT_CONTROLLER] = front_controller_device;
        controller_devices[BACK_CONTROLLER] = back_controller_device;
    
        //Load Parameters
        pnh.param("frontleft_pod_position_index", FL_POD, 0);
        pnh.param("frontright_pod_position_index", FR_POD, 1);
        pnh.param("backright_pod_position_index", BR_POD, 2);
        pnh.param("backleft_pod_position_index", BL_POD, 3);
        pnh.param("frontleft_pod_channel", POD_CHANNEL[FL_POD], 0);
        pnh.param("frontright_pod_channel", POD_CHANNEL[FR_POD], 1);
        pnh.param("backright_pod_channel", POD_CHANNEL[BR_POD], 0);
        pnh.param("backleft_pod_channel", POD_CHANNEL[BL_POD], 1);
        pnh.param("frontleft_pod_neutral_position", POD_POSITION_NEUTRAL[FL_POD], 0.0);
        pnh.param("frontright_pod_neutral_position", POD_POSITION_NEUTRAL[FR_POD], 0.0);
        pnh.param("backright_pod_neutral_position", POD_POSITION_NEUTRAL[BR_POD], 0.0);
        pnh.param("backleft_pod_neutral_position", POD_POSITION_NEUTRAL[BL_POD], 0.0);
        pnh.param("frontleft_pod_encoder_reverse", POD_ENCODER_REV[FL_POD], false);
        pnh.param("frontright_pod_encoder_reverse", POD_ENCODER_REV[FR_POD], false);
        pnh.param("backright_pod_encoder_reverse", POD_ENCODER_REV[BR_POD], false);
        pnh.param("backleft_pod_encoder_reverse", POD_ENCODER_REV[BL_POD], false);
        pnh.param("frontleft_pod_motor_reverse", POD_MOTOR_REV[FL_POD], false);
        pnh.param("frontright_pod_motor_reverse", POD_MOTOR_REV[FR_POD], false);
        pnh.param("backright_pod_motor_reverse", POD_MOTOR_REV[BR_POD], false);
        pnh.param("backleft_pod_motor_reverse", POD_MOTOR_REV[BL_POD], false);
        pnh.param("pod_output_torque_per_amp", OUTPUT_TORQUE_PER_AMP, 1.0);
        pnh.param("pod_motor_current_high_above", POD_MOTOR_CURRENT_HIGH_ABOVE, -1);
        
        //Setup pod actuator interfaces
        hardware_interface::ActuatorStateHandle state_handleFL("walrus/front_left_pod_joint_actuator", &pod_position[FL_POD], &pod_velocity[FL_POD], &pod_effort[FL_POD]);
        asi_.registerHandle(state_handleFL);
        hardware_interface::ActuatorStateHandle state_handleFR("walrus/front_right_pod_joint_actuator", &pod_position[FR_POD], &pod_velocity[FR_POD], &pod_effort[FR_POD]);
        asi_.registerHandle(state_handleFR);
        hardware_interface::ActuatorStateHandle state_handleBR("walrus/back_right_pod_joint_actuator", &pod_position[BR_POD], &pod_velocity[BR_POD], &pod_effort[BR_POD]);
        asi_.registerHandle(state_handleBR);
        hardware_interface::ActuatorStateHandle state_handleBL("walrus/back_left_pod_joint_actuator", &pod_position[BL_POD], &pod_velocity[BL_POD], &pod_effort[BL_POD]);
        asi_.registerHandle(state_handleBL);        
        hardware_interface::ActuatorHandle effort_handleFL(state_handleFL, &pod_effort_cmd[FL_POD]);
        aei_.registerHandle(effort_handleFL);
        hardware_interface::ActuatorHandle effort_handleFR(state_handleFR, &pod_effort_cmd[FR_POD]);
        aei_.registerHandle(effort_handleFR);
        hardware_interface::ActuatorHandle effort_handleBR(state_handleBR, &pod_effort_cmd[BR_POD]);
        aei_.registerHandle(effort_handleBR);
        hardware_interface::ActuatorHandle effort_handleBL(state_handleBL, &pod_effort_cmd[BL_POD]);
        aei_.registerHandle(effort_handleBL);
        
        //Add diagnostics updaters
        diagnostic_updater.add("Front Left Pod Control", boost::bind(&WalrusPodHW::pod_control_diagnostic_callback, this, _1, FL_POD));
        diagnostic_updater.add("Front Right Pod Control", boost::bind(&WalrusPodHW::pod_control_diagnostic_callback, this, _1, FR_POD));
        diagnostic_updater.add("Back Left Pod Control", boost::bind(&WalrusPodHW::pod_control_diagnostic_callback, this, _1, BL_POD));
        diagnostic_updater.add("Back Right Pod Control", boost::bind(&WalrusPodHW::pod_control_diagnostic_callback, this, _1, BR_POD));
        diagnostic_updater.add("Front Pod Controller", boost::bind(&WalrusPodHW::pod_control_diagnostic_callback, this, _1, FRONT_CONTROLLER));
        diagnostic_updater.add("Back Pod Controller", boost::bind(&WalrusPodHW::pod_control_diagnostic_callback, this, _1, BACK_CONTROLLER));
    }
    
    WalrusPodHW::WalrusPodHW~()
    {
        delete controllers[0];
        delete controllers[1];
    }
    
    bool WalrusPodHW::init()
    {        
        //Setup publishers and subscribers to communicate with the embedded board
        hs_feedback = nh_.subscribe("main_board/pod_motor_feedback", 1000, &WalrusPodHW::pm_feedback_callback, this);   

	    registerInterface(&asi_);
	    registerInterface(&aei_);
	    
	    controllers[FRONT_CONTROLLER]->open(front_controller_device);
        controllers[BACK_CONTROLLER]->open(back_controller_device);

	    std::vector<std::string> actuator_names = boost::assign::list_of
	      ("walrus/front_left_pod_joint_actuator")
	      ("walrus/front_right_pod_joint_actuator")
	      ("walrus/back_left_pod_joint_actuator")
	      ("walrus/back_right_pod_joint_actuator");
	    return loadTransmissions(actuator_names);
    }
       
    void WalrusPodHW::write(ros::Duration dt)
    {
        boost::lock_guard<boost::mutex> lock(feedback_mutex);
        robot_transmissions_.get<transmission_interface::JointToActuatorEffortInterface>()->propagate(); 
            
        //Send motor control message            
        for (int l = 0; l < 4; l++)
        {  
            if (pod_effort_cmd[l] < -1)
                pod_effort_cmd[l] = -1;
            else if (pod_effort_cmd[l] > 1)
	            pod_effort_cmd[l] = 1;
            controllers[l & CONTROLLER_MASK]->setPower(POD_CHANNEL[l], pod_effort_cmd[l] * (POD_MOTOR_REV[l] ? -1 : 1));
        }
        
        hs_control.publish(hs_control_msg);   
    }
    
    void WalrusPodHW::read(ros::Duration dt)
    {
        boost::lock_guard<boost::mutex> lock(feedback_data_mutex);
       
        for (int l = 0; l < 4; l++)
        {
            double position, delta;
            position = ((pod_raw_position[l]/1023.0)*2*M_PI)-M_PI; //Convert raw ADC  value (0-1023) to angle (-pi to pi)
                       
            position -= POD_POSITION_NEUTRAL[l];
            if (position < -M_PI)
                position += 2*M_PI;
            else if (position > M_PI)
                position -= 2*M_PI;
            if (POD_ENCODER_REV[l])
                position *= -1;
            delta = position - pod_position[l];
            if (delta > M_PI) //Decrease angle across 0
                pod_velocity[l] = (delta-2*M_PI)/dt.toSec();
            else if (delta < -M_PI) //Increase angle across 0
                pod_velocity[l] = (delta+2*M_PI)/dt.toSec();
            else  
                pod_velocity[l] = delta/dt.toSec();
            pod_position[l] = position;        
            controllers[l & CONTROLLER_MASK]->getCurrent(POD_CHANNEL[l], &pod_current[l])
            pod_effort[l] = pod_current[l] * OUTPUT_TORQUE_PER_AMP;
        }        
        
	    robot_transmissions_.get<transmission_interface::ActuatorToJointStateInterface>()->propagate();
    }
    
    void WalrusPodHW::update_diagnostics()
    {
        diagnostic_updater.update();
    }
    
    void WalrusPodHW::pm_feedback_callback(const walrus_firmware_msgs::MainBoardPodMotorFeedback& msg)
    {
        boost::lock_guard<boost::mutex> lock(feedback_mutex);
        for (int l = 0; l < 4; l++)
            pod_raw_position[l] = msg.pod_position[POD_POSITION_INDEX[l]];
        last_pm_feedback = ros::Time::now();
    }
    
    void WalrusPodHW::pod_control_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat, int pod)
    {
        bool position_valid = pm_feedback_timeout > (ros::Time::now() - last_pm_feedback);
        bool current_valid = controllers[pod & CONTROLLER_MASK]->is_connected();
    
        bool warning = false;
        bool error = false;
        string msg = "";   
   
        if (!position_valid)
        {
            error = true;
            msg += "No position data. ";
        }
        if (!current_valid)
        {
            error = true;
            msg += "No current data. ";
        }
        else if (POD_MOTOR_CURRENT_HIGH_ABOVE > 0 && pod_current[index] > POD_MOTOR_CURRENT_HIGH_ABOVE)
        {
            warning = true;
            msg += "Motor current high. ";            
        }     
        
        if (error)
            level = diagnostic_msgs::DiagnosticStatus::ERROR;
        else if (warning)
            level = diagnostic_msgs::DiagnosticStatus::WARN;
        else
            msg = "Everything OK";
        
        stat.summary(level, msg); 
        
        stat.add("Velocity", position_valid ? formatDouble(pod_velocity[index], 0) + " rpm" : "No Position Data");
        stat.add("Position", position valid ? formatDouble((pod_position[index]*180/M_PI), 2) + "\xc2\xb0" : "No Positon Data");
        stat.add("Current", current_valid ? formatDouble(pod_current[index], 3) + " A" : "No Current Data");
    }
    
    void WalrusPodHW::roboteq_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat, int controller)
    {
        if (controllers[controller]->is_connected())
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
        else
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Not Connected");
    }
    

}
