
#include <ros/ros.h>
#include <walrus_firmware_msgs/DiagnosticTXMsg.h>
#include <walrus_firmware_msgs/DiagnosticRXMsg.h>
#include <std_msgs/Empty.h>
#include "Screen.h"
#include "MenuScreen.h"
#include "MainScreen.h"
#include "ActionScreen.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/publisher.h"


ros::Publisher heartBeatTX;
ros::Publisher boardTX;
ros::Subscriber heartBeatRX;
ros::Subscriber boardRX;

MainScreen mainScreen = MainScreen(nh);

Screen currentScreen;

MenuScreen statusMenu;

ros::Time lastBeat;
ros::Time lastAction;

diagnostic_updater::Updater updater;
	

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
	MainScreen = mainScreen;
	MenuScreen mainMenu("Main Menu", &mainScreen);
	
	MenuScreen statusMenu("System Status", &mainMenu);
	statusMenuItem("System Status", statusMenu);
	
	SelectorMenuItem modeSelectMenu("Mode", &mainMenu);
	
	MenuScreen actionMenu("Actions:", &mainMenu);
	MenuItem actionMenuItem("Actions", &actionMenu);
	
	mainMenu.addItem(statusMenuItem);
	mainMenu.addItem(modeSelectMenu);
	mainMenu.addItem(actionMenuItem);
	
	currentScreen = MainScreen(nh, mainMenu);
}

int main(int arcv, char** argv)
{
	//Initialize ROS Node
	ros::init(argc, argv, "diagnosticboard_driver";

	//Get a node handle
	ros::NodeHandle nh;
	
	updater.setHardwareID("Embedded Diagnostics Board");
	
	
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

