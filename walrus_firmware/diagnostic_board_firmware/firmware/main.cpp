

#include <Arduino.h>
#include <ros.h>
#include <walrus_firmware_msgs/DiagnosticRXMsg.h>
#include <walrus_firmware_msgs/DiagnosticTXMsg.h>
#include "constants.h"
#include "Bounce2.h"
#include "LiquidCrystal.h"

void recv_msg(const walrus_firmware_msgs::DiagnosticTXMsg& msg);
 
//ROS node handle
ros::NodeHandle nh;
//ROS Message publisher
walrus_firmware_msgs::DiagnosticRXMsg rx_msg;
ros::Publisher rx("/walrus/diagnostic_board/rx", &rx_msg);
//ROS Message subscriber
ros::Subscriber<walrus_firmware_msgs::DiagnosticTXMsg> tx("/walrus/diagnostic_board/tx", &recv_msg);

//LCD Object
LiquidCrystal lcd(P_DISP_RS, P_DISP_RW, P_DISP_EN, P_DISP_D0, P_DISP_D1, P_DISP_D2, P_DISP_D3, P_DISP_D4, P_DISP_D5, P_DISP_D6, P_DISP_D7);
//Bounce objects to debounce buttons
Bounce up, right, down, left, cent;
//Booleans to remember if a button was pressed
bool last_up, last_right, last_down, last_cent, last_left;
//LED cycle time when spinning LEDs
int last_cycle = 0;
//LED that is currently on when spinning LEDs
int last_on = 0;


//Receive TX message from ROS master
void recv_msg(const walrus_firmware_msgs::DiagnosticTXMsg& msg)
{
	//msg.display contains an array of 80 characters in row major order
	//clear the screen and print them if they are not 0
	lcd.clear();
	for (int r = 0; r < 4; r++)
	{
		for (int c = 0; c < 20; c++)
		{
			if (msg.display[r*20+c] > 0)
			{
				lcd.setCursor(c, r);
				lcd.write(msg.display[r*20+c]);
			}
		}
	}
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
	
	//Setup LCD Object
	lcd.noBlink();
	lcd.noCursor();
	
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
	last_up = false;
	last_down = false;
	last_right = false;
	last_cent = false;
	last_left = false;
}



void loop()
{s
	//Make LED's solid if connected and spin them if not
	if (nh.connected())
	{
		digitalWrite(P_LED_UP, HIGH);
		digitalWrite(P_LED_RIGHT, HIGH);
		digitalWrite(P_LED_DOWN, HIGH);
		digitalWrite(P_LED_LEFT, HIGH);
		digitalWrite(P_LED_CENT, HIGH);
		digitalWrite(P_LED_STATUS, HIGH);
	}
	else
	{
		lcd.clear();
		lcd.print("Not Connected");
		digitalWrite(P_LED_STATUS, LOW);
		digitalWrite(P_LED_CENT, LOW);
		if (millis() > last_time + LED_CYCLE_TIME)
		{
			switch (last_on)
			{
				case 0:
					digitalWrite(P_LED_UP, HIGH);
					digitalWrite(P_LED_RIGHT, LOW);
					digitalWrite(P_LED_DOWN, LOW);
					digitalWrite(P_LED_LEFT, LOW);
					last_on = 0;
				break;
				case 1:
					digitalWrite(P_LED_UP, LOW);
					digitalWrite(P_LED_RIGHT, HIGH);
					digitalWrite(P_LED_DOWN, LOW);
					digitalWrite(P_LED_LEFT, LOW);
					last_on = 1;
				break;
				case 2:
					digitalWrite(P_LED_UP, LOW);
					digitalWrite(P_LED_RIGHT, LOW);
					digitalWrite(P_LED_DOWN, HIGH);
					digitalWrite(P_LED_LEFT, LOW);
					last_on = 2;
				break;
				case 3:
					digitalWrite(P_LED_UP, LOW);
					digitalWrite(P_LED_RIGHT, LOW);
					digitalWrite(P_LED_DOWN, LOW);
					digitalWrite(P_LED_LEFT, HIGH);
					last_on = 3;
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
	if (up.read() && !last_up)
	{
		last_up = true;
		rx_msg.button = 1;
		rx.publish(&rx_msg);
	}
	if (right.read() && !last_right)
	{
		last_down = true;
		rx_msg.button = 2;
		rx.publish(&rx_msg);
	}
	if (down.read() && !last_down)
	{
		last_up = true;
		rx_msg.button = 3;
		rx.publish(&rx_msg);
	}
	if (left.read() && !last_left)
	{
		last_left = true;
		rx_msg.button = 4;
		rx.publish(&rx_msg);
	}
	if (cent.read() && !last_cent)
	{
		last_cent = true;
		rx_msg.button = 5;
		rx.publish(&rx_msg);
	}
	
	//Clear button pressed bools when a button is released
	if (!up.read() && last_up)
		last_up = false;
	if (!right.read() && last_right)
		last_right = false;
	if (!down.read() && last_down)
		last_down = false;
	if (!left.read() && last_left)
		last_left = false;
	if (!cent.read() && last_cent)
		last_cent = false;
	
	//Check for ros updates
	nh.spinOnce();
}
