
#include <Arduino.h>
#include <ros.h>
#include <walrus_firmware_msgs/DiagnosticScreen.h>
#include <walrus_firmware_msgs/DiagnosticButton.h>
#include "constants.h"
#include "Bounce2.h"
#include "LiquidCrystal.h"

void recv_msg(const walrus_firmware_msgs::DiagnosticScreen& msg);
 
//ROS node handle
ros::NodeHandle nh;
//ROS Message publisher
walrus_firmware_msgs::DiagnosticButton rx_msg;
ros::Publisher rx("diagnostic_board/rx", &rx_msg);
//ROS Message subscriber
ros::Subscriber<walrus_firmware_msgs::DiagnosticScreen> tx("diagnostic_board/tx", &recv_msg);

//LCD Object
LiquidCrystal lcd(P_DISP_RS, P_DISP_RW, P_DISP_EN, P_DISP_D0, P_DISP_D1, P_DISP_D2, P_DISP_D3, P_DISP_D4, P_DISP_D5, P_DISP_D6, P_DISP_D7);
//Bounce objects to debounce buttons
Bounce up, right, down, left, cent;
//LED cycle time when spinning LEDs
long last_time = 0;
//LED that is currently on when spinning LEDs
int last_on = 0;
//Marks whether the node was connected on the last check
bool was_connected;



//Receive TX message from ROS master
void recv_msg(const walrus_firmware_msgs::DiagnosticScreen& msg)
{
	//msg.display contains an array of 80 characters in row major order
	//all characters in the array are required to be set (blanks are ascii spaces)
	//return the lcd to home and print them
	lcd.home();
	lcd.write(msg.display, 80);
}

void setup()
{
	//Initialize node, publishers and subscribers
	nh.initNode();
	nh.advertise(rx);	
	nh.subscribe(tx);

	//Setup LED pins as outputs and set them high
	pinMode(P_LED_UP, OUTPUT);
	pinMode(P_LED_RIGHT, OUTPUT);
	pinMode(P_LED_DOWN, OUTPUT);
	pinMode(P_LED_LEFT, OUTPUT);
	pinMode(P_LED_CENT, OUTPUT);
	pinMode(P_LED_STATUS, OUTPUT);
	pinMode(P_BUTT_UP, INPUT_PULLUP);
	pinMode(P_BUTT_RIGHT, INPUT_PULLUP);
	pinMode(P_BUTT_DOWN, INPUT_PULLUP);
	pinMode(P_BUTT_LEFT, INPUT_PULLUP);
	pinMode(P_BUTT_CENT, INPUT_PULLUP);
	pinMode(P_DISP_D0, OUTPUT);
	pinMode(P_DISP_D1, OUTPUT);
	pinMode(P_DISP_D2, OUTPUT);
	pinMode(P_DISP_D3, OUTPUT);
	pinMode(P_DISP_D4, OUTPUT);
	pinMode(P_DISP_D5, OUTPUT);
	pinMode(P_DISP_D6, OUTPUT);
	pinMode(P_DISP_D7, OUTPUT);
	
	//Setup LCD Object
	lcd.begin(20,4);
	lcd.createChar(walrus_firmware_msgs::DiagnosticScreen::BATT_0, BATT_0_CHAR);
	lcd.createChar(walrus_firmware_msgs::DiagnosticScreen::BATT_20, BATT_20_CHAR);
	lcd.createChar(walrus_firmware_msgs::DiagnosticScreen::BATT_40, BATT_40_CHAR);
	lcd.createChar(walrus_firmware_msgs::DiagnosticScreen::BATT_60, BATT_60_CHAR);
	lcd.createChar(walrus_firmware_msgs::DiagnosticScreen::BATT_80, BATT_80_CHAR);
	lcd.createChar(walrus_firmware_msgs::DiagnosticScreen::BATT_100, BATT_100_CHAR);
	lcd.createChar(walrus_firmware_msgs::DiagnosticScreen::UP_ARROW, UP_ARROW_CHAR);
	lcd.createChar(walrus_firmware_msgs::DiagnosticScreen::DOWN_ARROW, DOWN_ARROW_CHAR);
	
	//Setup Buttons
	up.interval(DEBOUNCE_TIME);
	down.interval(DEBOUNCE_TIME);
	right.interval(DEBOUNCE_TIME);
	left.interval(DEBOUNCE_TIME);
	cent.interval(DEBOUNCE_TIME);
	up.attach(P_BUTT_UP);
	down.attach(P_BUTT_DOWN);
	right.attach(P_BUTT_RIGHT);
	left.attach(P_BUTT_LEFT);
	cent.attach(P_BUTT_CENT);
	
	//Mark as connected so not connected message is displayed
	was_connected = true;
}

void loop()
{
	//Make LED's solid if connected and spin them if not
	if (nh.connected())
	{
		digitalWrite(P_LED_UP, HIGH);
		digitalWrite(P_LED_RIGHT, HIGH);
		digitalWrite(P_LED_DOWN, HIGH);
		digitalWrite(P_LED_LEFT, HIGH);
		digitalWrite(P_LED_CENT, HIGH);
		digitalWrite(P_LED_STATUS, HIGH);
		was_connected = true;
	}
	else if (was_connected)
	{
		lcd.clear();
		lcd.home();
		lcd.print("Waiting for PC...");
		lcd.write((uint8_t)0);
		lcd.write(1);
		lcd.write(2);
		lcd.write(3);
		lcd.write(4);
		lcd.write(5);
		lcd.write(6);
		lcd.write(7);
		was_connected = false;
		digitalWrite(P_LED_STATUS, LOW);
		digitalWrite(P_LED_CENT, HIGH);
	}
	else
	{
		if (millis() > last_time + LED_CYCLE_SPEED)
		{
			switch (last_on)
			{
				case 0:
					digitalWrite(P_LED_UP, HIGH);
					digitalWrite(P_LED_RIGHT, LOW);
					analogWrite(P_LED_DOWN, LED_SECOND_DIMM);
					analogWrite(P_LED_LEFT, LED_FIRST_DIMM);
					last_on = 1;
				break;
				case 1:
					analogWrite(P_LED_UP, LED_FIRST_DIMM);
					digitalWrite(P_LED_RIGHT, HIGH);
					digitalWrite(P_LED_DOWN, LOW);
					analogWrite(P_LED_LEFT, LED_SECOND_DIMM);
					last_on = 2;
				break;
				case 2:
					analogWrite(P_LED_UP, LED_SECOND_DIMM);
					analogWrite(P_LED_RIGHT, LED_FIRST_DIMM);
					digitalWrite(P_LED_DOWN, HIGH);
					digitalWrite(P_LED_LEFT, LOW);
					last_on = 3;
				break;
				case 3:
					digitalWrite(P_LED_UP, LOW);
					analogWrite(P_LED_RIGHT, LED_SECOND_DIMM);
					analogWrite(P_LED_DOWN, LED_FIRST_DIMM);
					digitalWrite(P_LED_LEFT, HIGH);
					last_on = 0;
				break;
			}
			last_time = millis();
		}
	}
	
	
	//Update all buttons
	up.update();
	down.update();
	right.update();
	left.update();
	cent.update();
	
	//If button is pressed, publish a message
	if (up.fell())
	{
		rx_msg.button = 1;
		rx.publish(&rx_msg);
	}
	if (right.fell())
	{
		rx_msg.button = 2;
		rx.publish(&rx_msg);
	}
	if (down.fell())
	{
		rx_msg.button = 3;
		rx.publish(&rx_msg);
	}
	if (left.fell())
	{
		rx_msg.button = 4;
		rx.publish(&rx_msg);
	}
	if (cent.fell())
	{
		rx_msg.button = 5;
		rx.publish(&rx_msg);
	}
	
	//Check for ros updates
	nh.spinOnce();
}

