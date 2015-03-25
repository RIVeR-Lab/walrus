#include "ros/ros.h"
#include "walrus_interface_controller/InterfaceControllerState.h"
#include "walrus_joystick_controller/JoystickControllerState.h"
#include <std_msgs/Bool.h>


bool first_update = true;
int last_active_controller = -1;
std::vector<std::string> controllers;
std::vector<bool> controller_last_enable;
std::vector<ros::Publisher> controller_enable_pubs;
std::vector<ros::Subscriber> controller_state_subs;
std::vector<walrus_interface_controller::InterfaceControllerState> controller_last_state;

ros::Timer update_timer;

void updateActiveController() {
  int activate_controller = -1;
  for(unsigned int i = 0; i < controllers.size(); ++i) {
    if(controller_last_state[i].state == walrus_interface_controller::InterfaceControllerState::DISABLED ||
       controller_last_state[i].state == walrus_interface_controller::InterfaceControllerState::ACTIVE) {
      activate_controller = i;
      break;
    }
  }
  for(unsigned int i = 0; i < controllers.size(); ++i) {
    bool enable = (i == activate_controller);

    if(controller_last_enable[i] != enable) {
      std_msgs::Bool enable_msg;enable_msg.data = enable;
      controller_enable_pubs[i].publish(enable_msg);
      controller_last_enable[i] = enable;
    }
  }
  if(last_active_controller != activate_controller || first_update) {
    if(activate_controller == -1)
      ROS_WARN_STREAM("No available controllers");
    else
      ROS_INFO_STREAM("Activated controller: " << controllers[activate_controller]);
  }
  last_active_controller = activate_controller;
  first_update = false;
}

void controller_state_callback(const walrus_interface_controller::InterfaceControllerState::ConstPtr& state, int index) {
  controller_last_state[index] = *state;
  updateActiveController();
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "interface_manager");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.getParam("controllers", controllers);


  for(unsigned int i = 0; i < controllers.size(); ++i) {
    ROS_INFO_STREAM("Loaded controller: " << controllers[i]);
    ros::NodeHandle controller_nh(nh, controllers[i]);
    controller_state_subs.push_back(controller_nh.subscribe<walrus_interface_controller::InterfaceControllerState>("state", 1, boost::bind(controller_state_callback, _1, i)));
    controller_enable_pubs.push_back(controller_nh.advertise<std_msgs::Bool>("enable", 1, true));
    walrus_interface_controller::InterfaceControllerState state;
    state.header.stamp = ros::Time::now();
    state.state = walrus_interface_controller::InterfaceControllerState::UNAVAILABLE;
    controller_last_state.push_back(state);
  }
  controller_last_enable.resize(controllers.size());
  updateActiveController();
  update_timer = nh.createTimer(ros::Duration(1.0), boost::bind(updateActiveController));

  ros::spin();
}
