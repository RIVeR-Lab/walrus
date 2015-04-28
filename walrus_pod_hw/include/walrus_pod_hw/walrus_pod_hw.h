#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <walrus_base_hw/walrus_robot_base.h>
#include <walrus_firmware_msgs/MainBoardPodMotorFeedback.h>
#include <hardware_interface/hardware_interface.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <string>
#include <roboteq_driver/roboteq_motor_controller.h>

namespace walrus_pod_hw 
{

    class WalrusPodHW : public walrus_base_hw::WalrusRobotBase
    {
    public:
       WalrusPodHW(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::string front_controller_device, std::string back_controller_device);
       
       ~WalrusPodHW();
       
       bool init();
       void write(ros::Duration dt);
       void read(ros::Duration dt);
       void update_diagnostics();
     
    private:  
       diagnostic_updater::Updater diagnostic_updater;
       
       hardware_interface::ActuatorStateInterface asi_;
       hardware_interface::EffortActuatorInterface aei_;
       
       std::string controller_devices[2];
       roboteq_driver::RoboteqMotorController* controllers[2];
       
       void pm_feedback_callback(const walrus_firmware_msgs::MainBoardPodMotorFeedback& msg);
       
       static std::string formatDouble(double value, int precision);
       
       void pod_control_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat, int pod);
       void roboteq_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper &stat, int controller);
       
       ros::Subscriber pm_feedback;
            
       boost::mutex feedback_mutex;
       
       double pod_velocity[4], pod_raw_position[4], pod_position[4], pod_current[4], pod_effort[4], pod_effort_cmd[4];
       ros::Time last_pm_feedback;
       const ros::Duration pm_feedback_timeout;    
       
            
       //Pod constants
       const int FL_POD, BL_POD, FR_POD, BR_POD;
       const int CONTROLLER_MASK;
       const int FRONT_CONTROLLER, BACK_CONTROLLER;
       int POD_POSITION_INDEX[4];
       int POD_CHANNEL[4];
       double POD_POSITION_NEUTRAL[4];
       bool POD_ENCODER_REV[4];
       bool POD_MOTOR_REV[4];
       double OUTPUT_TORQUE_PER_AMP; 
       double POD_MOTOR_CURRENT_HIGH_ABOVE;
       double POD_MOTOR_LIMIT;
 
    };

}


