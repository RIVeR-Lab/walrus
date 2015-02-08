/*
This is an Arduino driver that implements the SMBus protocol for communicating 
with BB2590 batteies.
Author: Brian Eccles
*/

#ifndef SMARTBATT_H
#define SMARTBATT_H

#include <Arduino.h>
#include <avr/io.h>
#include "i2c_buses.h"

class SmartBatt
{
private:
	//SMBus i2c structure
	i2c_bus_t* i2c_bus;
	//True if begin has been called
	bool started;
	
	
	//Private bus functions
	int read_block(i2c_bus_t* bus, byte addr, byte command_code, byte* buf, int buf_size);
	int read_word(i2c_bus_t* bus, byte addr, byte command_code);

public:
	SmartBatt();
	
	//Setup this object, must be called before using other functions
	void begin(i2c_bus_t* i2c_bus);
	
	//Get the battery output voltage in mV
	int getVoltage();
	//Get the battery output current in mA
	int getCurrent();
	//Get the battery output charge in hundredths of a percent (0.01%)
	int getCharge();
	//Get the battery temperature in hundredths of a degree C
	int getTemp();
	//Get manufacturer name
	void getManufacturer(char* buf, int len);
	//Get device name
	void getDeviceName(char* buf, int len);
	//Get device chemistry
	void getChemistry(char* buf, int len);
	//Get serial number
	int getSerial();
	//Get average current
	int getAvgCurrent();
	//Get remaining capacity
	int getRemCap();
	//Get full capacity
	int getFullCap();
};

#endif
