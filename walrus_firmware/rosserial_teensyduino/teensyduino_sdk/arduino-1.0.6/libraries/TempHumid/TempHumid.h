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

#define CONFIG_BYTE 0x00
#define MEASURE_TEMP_CMD_HOLD 0xE3
#define MEASURE_HUMID_CMD_HOLD 0xE5
#define MEASURE_TEMP_CMD_NOHOLD 0xF3
#define MEASURE_HUMID_CMD_NOHOLD 0xF5
#define READ_TEMP 0xE0
#define WRITE_REG_CMD 0xE6

class TempHumid
{
	
private:
	//I2C Slave Address
	int address;
	//True if begin has been called
	bool started;
	//Stores values when read is called
	int humidity;
	int temp;
		
public:
	TempHumid();
	
	//Setup this object, must call before using get functions
	//address - Slave address of device (check datasheet)
	void begin(int address);
	
	//Issue a command to measure humidity and temperature, follow by a call to readAll at least 20ms later
	void measure();
	
	//Reads humidity and temperature from the device, measure must be called about 20ms prior to calling this
	//Returns true if reading succeeded
	bool readAll();
	
	//Get the temperature value read by readAll() as a 16 bit integer in hundredths of degrees C
	int getTemp();
	
	//Get the humidity value read by readAll() as a 16 bit integer in hundredths of a percent
	int getHumidity();
	
	//Returns temperature as a 16 bit integer in hundredths of degrees C, blocks during measurment (~3ms)
	int getTempNow();
	
	//Returns relative humidity as a 16 bit integer in hundredths of a percent, blocks during measurement (~20ms)
	int getHumidityNow();
	
};

#endif
