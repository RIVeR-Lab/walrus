
#include <ros/ros.h>
#include <walrus_firmware_msgs/DiagnosticTXMsg.h>
#include <walrus_firmware_msgs/DiagnosticRXMsg.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <std_msgs/Empty.h>
#include "Screen.h"
#include "MenuScreen.h"
#include "MainScreen.h"
#include "ActionScreen.h"


ros::Publisher heartBeatTX;
ros::Publisher boardTX;
ros::Subscriber heartBeatRX;
ros::Subscriber boardRX;

MainScreen mainScreen = MainScreen(nh);

Screen currentScreen;

MenuScreen mainMenu;

ros::Time lastBeat;
	

void OnBoardRX(walrus_firmware_msgs::DiagnosticRXMsg msg)
{
	if (msg.button >= 1 && msg.button <= 5
		currentScreen = currentScreen.doAction(msg.button);
	else
		///This is  bad mmmmmkay
}

void OnHeartbeat(std_msgs::Empty msg)
{
	lastBeat = ros::Time::now();
}

void generateMenus()
{
	MenuScreen mainMenu;
	
	
	currentScreen = MainScreen(nh, mainMenu);
}

int main(int arcv, char** argv)
{
	//Initialize ROS Node
	ros::init(argc, argv, "diagnosticboard_driver";

	//Get a node handle
	ros::NodeHandle nh;
	
	generateMenus();
	
	int counter = 0;
	ros::Rate rate(100);
	while (ros::ok())
	{
		counter++;
		if (counter % 5 == 0)
			//Send heartbeat
		if 
		ros::SpinOnce();
		r.sleep();
	}
}

