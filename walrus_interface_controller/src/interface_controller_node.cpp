#include "ros/ros.h"
#include "walrus_interface_controller/InterfaceControllerState.h"
#include "walrus_joystick_controller/JoystickControllerState.h"


ros::Publisher state_pub;
ros::Timer state_pub_timer;

bool have_joystick_controller;
ros::Subscriber joystick_sub;
ros::Publisher joystick_enable_pub;
walrus_joystick_controller::JoystickControllerState::ConstPtr joystick_state;


void updateState() {
  walrus_interface_controller::InterfaceControllerState state;
  bool available = !have_joystick_controller || (joystick_state && joystick_state->available);
  bool active = !have_joystick_controller || (joystick_state && joystick_state->enabled);
  if(available) {
    if(active) {
      state.state = walrus_interface_controller::InterfaceControllerState::ACTIVE;
    }
    else {
      state.state = walrus_interface_controller::InterfaceControllerState::DISABLED;
    }
  }
  else {
    state.state = walrus_interface_controller::InterfaceControllerState::UNAVAILABLE;
  }
  state.header.stamp = ros::Time::now();
  state_pub.publish(state);
}

void joystick_state_callback(const walrus_joystick_controller::JoystickControllerState::ConstPtr& state) {
  joystick_state = state;
  updateState();
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "interface_controller");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param<bool>("have_joystick_controller", have_joystick_controller, false);

  state_pub = nh.advertise<walrus_interface_controller::InterfaceControllerState>("state", 1, true);
  state_pub_timer = nh.createTimer(ros::Duration(1.0), boost::bind(updateState));

  if(have_joystick_controller)
    joystick_sub = nh.subscribe<walrus_joystick_controller::JoystickControllerState>("joystick_controller/state", 1, joystick_state_callback);

  ros::spin();
}
