
#include <Arduino.h>
#include <ros.h>
#include <walrus_firmware_msgs/MainBoardTXMsg.h>
#include <walrus_firmware_msgs/MainBoardRXMsg.h>
#include "constants.h"
#include <Wire.h>
#include <TempHumid.h>
#include <MPL3115A2.h>
#include <ExternalADC.h>
#include <OneWire.h>
#include <Servo.h>
#include <SmartBatt.h>

void recv_msg(const walrus_firmware_msgs::MainBoardTXMsg& msg);
void disable();

//ROS node handle
ros::NodeHandle nh;
//ROS Message publisher
walrus_firmware_msgs::MainBoardRXMsg rx_msg;
ros::Publisher rx("/walrus/main_board/rx", &rx_msg);
//ROS Message subscriber
ros::Subscriber<walrus_firmware_msgs::MainBoardTXMsg> tx("/walrus/main_board/tx", &recv_msg);

//Pressure sensor
MPL3115A2 pressure_sense;
//External ADC
ExternalADC extADC;
//Temperature and humidity sensor
TempHumid temphumid_sense;
//OneWire object for temperature sensors
OneWire exttemp_sense(P_EXT_TEMP);
//Motor servos
Servo motor1, motor2, motor3, motor4;
//Battery SMBus objects
SmartBatt upper[4], lower[4];
//Time in milliseconds of the last received message
long last_msg = 0;
//Stores values read from external temperature sensors
int temps[10] = {0,0,0,0,0,0,0,0,0,0};
//Temp sensor addresses
uint8_t temp_addr[10][8] = {TEMP_1_ADDR, TEMP_2_ADDR, TEMP_3_ADDR, TEMP_4_ADDR, TEMP_5_ADDR, TEMP_6_ADDR, TEMP_7_ADDR, TEMP_8_ADDR, TEMP_9_ADDR, TEMP_10_ADDR};
bool temp_en[10] = {TEMP_1_EN, TEMP_2_EN, TEMP_3_EN, TEMP_4_EN, TEMP_5_EN, TEMP_6_EN, TEMP_7_EN, TEMP_8_EN, TEMP_9_EN, TEMP_10_EN};
//Temp sensor buffer;
uint8_t temp_buff[9];
//Rate to control main loop
long loop_start;
//Count the number of loops for timing
long counter = 1;
//Status of system (to show on LED)
int status = STATUS_NO_PC;
bool led_state = false;

//Receive TX message from ROS master
void recv_msg(const walrus_firmware_msgs::MainBoardTXMsg &msg)
{
	//Set motor speeds 
	motor1.writeMicroseconds(msg.motor_power[1]);
	motor2.writeMicroseconds(msg.motor_power[2]);
	motor3.writeMicroseconds(msg.motor_power[3]);
	motor4.writeMicroseconds(msg.motor_power[4]);
	//Set contactor power state
	if (msg.power_off)
		digitalWrite(P_CONTACTOR, HIGH);
	else
		digitalWrite(P_CONTACTOR, LOW);
	//Set external LED intensities
	analogWrite(P_EXT_LED_1, msg.LED_intensity[0]);
	analogWrite(P_EXT_LED_2, msg.LED_intensity[1]);
	analogWrite(P_EXT_LED_3, msg.LED_intensity[2]);
	//Mark receive time
	last_msg = millis();
}

//Stop all motors, turn of LEDs and keep power on
void disable()
{
	//Disable all motors
	motor1.writeMicroseconds(1500);
	motor1.writeMicroseconds(1500);
	motor1.writeMicroseconds(1500);
	motor1.writeMicroseconds(1500);
	//Keep contactor on
	digitalWrite(P_CONTACTOR, LOW);
	//Turn LED's off
	analogWrite(P_EXT_LED_1, 0);
	analogWrite(P_EXT_LED_2, 0);
	analogWrite(P_EXT_LED_3, 0);
}

void setup()
{
	//Initialize node, publishers and subscribers
	nh.initNode();
	nh.advertise(rx);
	nh.subscribe(tx);
	
	//Setup Motors
	motor1.attach(P_MOTOR_1);
	motor2.attach(P_MOTOR_2);
	motor3.attach(P_MOTOR_3);
	motor4.attach(P_MOTOR_4);
	
	//Setup digital IO
	pinMode(P_LED_STATUS, OUTPUT);
	pinMode(P_WATER_1, INPUT);
	pinMode(P_WATER_2, INPUT);
	pinMode(P_WATER_3, INPUT);
	pinMode(P_WATER_4, INPUT);
	pinMode(P_WATER_5, INPUT);
	pinMode(P_WATER_6, INPUT);
	pinMode(P_EXT_LED_1, OUTPUT);
	pinMode(P_EXT_LED_2, OUTPUT);
	pinMode(P_EXT_LED_3, OUTPUT);
	
	//Setup external ADC
	extADC.begin(ADDR_EXT_ADC, 2);
	
	//Setup temperature/humidity sensor
	temphumid_sense.begin(ADDR_TEMP_HUMID);
	
	//Setup Pressure sensor
	pressure_sense.begin();
	pressure_sense.setModeBarometer();
	pressure_sense.setOversampleRate(7); 
	pressure_sense.enableEventFlags();
	
	//Setup battery SMBus objects
	upper[0].begin(i2c_bus0);
	upper[1].begin(i2c_bus1);
	upper[2].begin(i2c_bus2);
	upper[3].begin(i2c_bus3);
	lower[0].begin(i2c_bus4);
	lower[1].begin(i2c_bus5);
	lower[2].begin(i2c_bus6);
	lower[3].begin(i2c_bus7);
	
	//Issue a one-wire convert command
	exttemp_sense.reset();
	exttemp_sense.skip();
	exttemp_sense.write(T_CONVERT);
	
}

void loop()
{	
	loop_start = millis();
	
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
		status = STATUS_OK;	
	//Blink status LED based on status
	if (counter % (status/ROS_MSG_RATE) == 0)
	{
		led_state = !led_state;
		digitalWrite(P_LED_STATUS, led_state);
	}
	
	//Read temperature
	if (counter % (TEMP_READ_RATE/ROS_MSG_RATE) == 0)
	{
		//Get data from temp sensors
		for (int l = 0; l < 10; l++)
		{
			if (temp_en[l])
			{
				exttemp_sense.reset();
				exttemp_sense.select(temp_addr[l]);
				exttemp_sense.write(READ_SCRATCH);
				for (int m = 0; m < 9; m++)
					temp_buff[m] = exttemp_sense.read();
				temps[l] = (temp_buff[1] << 8) | temp_buff[0];
			}
		}
		//Issue new convert command to all devices
		exttemp_sense.reset();
		exttemp_sense.skip();
		exttemp_sense.write(T_CONVERT);
	}
	//Read in external ADC samples
	extADC.sustain();
	
	//Read in motor currents
	rx_msg.motor_current[0] = analogRead(P_CURRENT_1);
	rx_msg.motor_current[1] = analogRead(P_CURRENT_2);
	rx_msg.motor_current[2] = analogRead(P_CURRENT_3);
	rx_msg.motor_current[3] = analogRead(P_CURRENT_4);
	//Read in motor encoders
	rx_msg.pod_position[0] = analogRead(P_ENCODER_1);
	rx_msg.pod_position[1] = analogRead(P_ENCODER_2);
	rx_msg.pod_position[2] = analogRead(P_ENCODER_3);
	rx_msg.pod_position[3] = analogRead(P_ENCODER_4);	
	//Read in external temperature
	for (int l = 0; l < 10; l++)
		rx_msg.temp_sense[l] = temps[l];
	//Read in water sensors
	rx_msg.water_sense[0] = digitalRead(P_WATER_1);
	rx_msg.water_sense[1] = digitalRead(P_WATER_2);
	rx_msg.water_sense[2] = digitalRead(P_WATER_3);
	rx_msg.water_sense[3] = digitalRead(P_WATER_4);
	rx_msg.water_sense[4] = digitalRead(P_WATER_5);
	rx_msg.water_sense[5] = digitalRead(P_WATER_6);	
	//Read in tension pots
	rx_msg.tension[0] = extADC.getValue(CHAN_POT1);
	rx_msg.tension[1] = extADC.getValue(CHAN_POT2);
	//Read in pressure 
	rx_msg.pressure = pressure_sense.readPressure();
	//Read in board temperature
	rx_msg.board_temp = temphumid_sense.getTemp();
	//Read in humidity
	rx_msg.humidity = temphumid_sense.getHumidity();	
	//Read in SMBUS values	
	for (int l = 0; l < 4; l++)
	{
		rx_msg.ucell_voltage[l] = upper[l].getVoltage();
		rx_msg.ucell_current[l] = upper[l].getCurrent();
		rx_msg.ucell_charge[l] = upper[l].getCharge();
		rx_msg.ucell_temp[l] = upper[l].getTemp();
		rx_msg.ucell_shutdown[l] = 0; //(upper[l].getShutdown() ? 1 : 0);
		rx_msg.lcell_voltage[l] = lower[l].getVoltage();
		rx_msg.lcell_current[l] = lower[l].getCurrent();
		rx_msg.lcell_charge[l] = lower[l].getCharge();
		rx_msg.lcell_temp[l] = lower[l].getTemp();
		rx_msg.lcell_shutdown[l] = 0; //(lower[l].getShutdown() ? 1 : 0);
	}
	//Publish our message
	rx.publish(&rx_msg);
	
	//Allow ros to receive
	nh.spinOnce();	
	//Sleep until next cycle
	delay(ROS_MSG_RATE-(millis()-loop_start));
	//Increment loop counter
	counter++;
}
