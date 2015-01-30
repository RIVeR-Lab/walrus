
#include <Arduino.h>
#include <ros.h>
#include <walrus_firmware_msgs/BoomBoardTXMsg.h>
#include <walrus_firmware_msgs/BoomBoardRXMsg.h>
#include "constants.h"
#include <TempHumid.h>
#include <ExternalADC.h>
#include <Bridge.h>
#include <Maxon.h>

void recv_msg(const walrus_firmware_msgs::BoomBoardTXMsg& msg);
void disable();

//ROS node handle
ros::NodeHandle nh;
//ROS Message publisher
walrus_firmware_msgs::BoomBaordRXMsg rx_msg;
ros::Publisher rx("walrus/main_board/rx", &rx_msg);
//ROS Message subscriber
ros::Subscriber<walrus_firmware_msgs::BoomBoardTXMsg> tx("/walrus/boom_board/tx", &recv_msg);

//External ADC
ExternalADC extADC
//Temperature and humididty sensor
TempHumid temphumid_sense;
//Maxon deploy motor controller
Maxon maxon;
//H-Bridge pan and tilt motor driver
Bridge bridge;
//Time in millisencods of the last received message
long last_msg = 0; 
//Rate to control main loop
ros::Rate = r(ROS_MSG_RATE);
//Count the number of loops for timing
long counter = 1;
//Status of system (to show on LED)
int status = STATUS_NO_PC;
bool led_state = false;


//Receive TX message from ROS master
void recv_msg(const walrus_firmware_msgs::BoomBoardTXMsg &msg)
{
	//Set motor speeds
	bridge.setMotor(CHAN_PAN_MOTOR, msg.pan_power);
	bridge.setMotor(CHAN_TILT_MOTOR, msg.tilt_power);
	maxon.setMotor(msg.deploy_power);
	//Set cam LED intensity
	analogWrite(P_CAM_LED, msg.LED_intensity);
	//Mark receive time
	last_msg = millis();
}

//Stop all motors and turn off cam LED
void disable()
{
	//Disable all motors
	bridge.setMotor(CHAN_PAN_MOTOR, 0);
	bridge.setMotor(CHAN_TILT_MOTOR, 0);
	bridge.disable(CHAN_PAN_MOTOR);
	bridge.disable(CHAN_TILT_MOTOR);
	maxon.setMotor(0);
	maxon.disable();
	//Turn CAM LED off
	analogWrite(P_CAM_LED, 0);
}

void setup()
{
	//Initialize node, publishers and subscribers
	nh.initNode();
	nh.advertise(rx);
	nh.subscribe(tx);
	
	//Setup digital IO
	pinMode(P_LED_STATUS, OUTPUT);
	pinMode(P_CAM_LED, OUTPUT);
	
	//Setup external ADC
	extADC.begin(ADDR_EXT_ADC, 9);
	
	//Setup temperature/humidity sensor
	temphumid_sense.begin(ADDR_TEMP_HUMID);
	
	//Setup maxon motor controller
	maxon.begin(P_MAXON_IN1, P_MAXON_IN2, P_MAXON_DIR, P_MAXON_EN, P_MAXON_SPEED, P_MAXON_READY, P_MAXON_STATUS);
	maxon.setMode(SPEED_MODE_OPEN)
	maxon.setLEDDir(1);
	
	//Setup H-bridge motor controller
	bridge.begin(P_BRIDGE_IN1, P_BRIDGE_IN2, P_BRIDGE_IN3, P_BRIDGE_IN4, P_BRIDGE_D1, P_BRIDGE_D2, P_BRIDGE_D3, P_BRIDGE_D4, P_BRIDGE_SFA, P_BRIDGE_SFB, P_BRIDGE_LEDA, P_BRIDGE_LEDB);
	bridge.setLEDDir(1);
	bridge.setBrake(CHAN_PAN_MOTOR);
	bridge.setBrake(CHAN_TILT_MOTOR);
	
}

void loop()
{
	//Disable motors if we lose PC connection
	if (!nh.connected())
	{
		disable();
		status = STATUS_NO_PC;
	}
	//Disable motors if we don't receive a message in a specific amount of time
	else if (millis() > last_msg + MOTOR_OFF_TIMEOUT)
	{
		disable();
		status = STATUS_NO_MSG;
	}
	else
	{
		bridge.enable(CHAN_PAN_MOTOR);
		bridge.enable(CHAN_TILT_MOTOR);
		maxon.enable();
		status = STATUS_OK;	
	}
	//Blink status LED based on status
	if (counter % (status/ROS_MSG_RATE) == 0)
	{
		led_state = !led_state;
		digital_write(P_LED_STATUS, led_state);
	}
	
	//Read in external ADC samples
	extADC.sustain();
	//Sustain maxon led status
	maxon.sustain();
	//Sustain H-bridge led status
	bridge.sustain();
	
	//Read in analog expansion samples
	rx_msgs.analog[0] = analogRead(P_ANALOG_EXP_0);
	rx_msgs.analog[1] = analogRead(P_ANALOG_EXP_1);
	rx_msgs.analog[2] = analogRead(P_ANALOG_EXP_2);
	rx_msgs.analog[3] = analogRead(P_ANALOG_EXP_3);
	//Read in potentiometers
	rx_msgs.deploy_position = extADC.getValue(CHAN_DEPLOY_POT);
	rx_msgs.tilt_position = extADC.getValue(CHAN_TILT_POT);
	rx_msgs.pan_position = extADC.getValue(CHAN_DEPLOY_POT);
	//Read in gas sensors
	rx_msgs.CO_sense = extADC.getValue(CHAN_CO_SENSE);
	rx_msgs.LPG_sense = extADC.getValue(CHAN_LPG_SENSE);
	rx_msgs.H_sense = extADC.getValue(CHAN_H_SENSE);
	rx_msgs.CNG_sense = extADC.getValue(CHAN_CNG_SENSE);
	//Read in temperature
	rx_msgs.temp = temphumid_sense.getTemp();
	rx_msgs.humidity = temphumid_sense.getHumidity();
	//Read in motor currents
	rx_msgs.pan_current = extADC.getValue(CHAN_PAN_CURRENT);
	rx_msgs.tilt_current = extADC.getValue(CHAN_TILT_CURRENT);
	//Publish our message
	rx.publish(&rx_msg);
	
	//Allow ros to receive
	nh.spinOnce();
	//Sleep until next cycle
	r.sleep();
	//Increment loop counter
	counter++;	
}


