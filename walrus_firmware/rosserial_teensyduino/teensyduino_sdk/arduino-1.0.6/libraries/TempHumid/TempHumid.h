/*
This is an Arduino driver for the Silicon Labs Si7020-A10 Humidiy
and Temperature sensor. It utilizes the standard Arduino Wire library
for I2C communications.
Author: Brian Eccles
*/

#ifndef TEMPHUMID_H
#define TEMPHUMID_H

#include <Arduino.h>
#include <Wire.h>

#define CONFIGURATION_BYTE 0x00
#define MEASURE_TEMP_CMD 0xE3
#define MEASURE_HUMID_CMD 0xE5
#define WRITE_REG_CMD 0xE6
#define READ_TIMEOUT 500

class TempHumid
{
	
private:
	//I2C Slave Address
	int address;
	//True if begin has been called
	bool started;
		
public:
	TempHumid();
	
	//Setup this object, must call before using get functions
	//address - Slave address of device (check datasheet)
	void begin(int address);
	
	//Returns temperature as a 16 bit integer in hundredths of degrees C
	int getTemp();
	
	//Returns relative humidity as a 16 bit integer in hundredths of a percent
	int getHumidity();
	
}

#endif