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
#include <string.h>

class SmartBatt
{
private:
    //SMBus i2c structure
    i2c_bus_t* i2c_bus;
    //True if begin has been called
    bool started;
    
    //Private bus functions
    int read_block(i2c_bus_t* bus, byte addr, byte command_code, byte* buf, int buf_size);
    bool read_word(i2c_bus_t* bus, byte addr, byte command_code, int16_t* value);

public:
    SmartBatt();
    
    //Setup this object, must be called before using other functions
    void begin(i2c_bus_t* i2c_bus);
    
    //Get the battery output voltage in mV
    bool getVoltage(int16_t* value);
    //Get the battery output current in mA
    bool getCurrent(int16_t* value);
    //Get the battery charge in hundredths of a percent (0.01%)
    bool getCharge(int16_t* value);
    //Get the battery temperature in hundredths of a degree C
    bool getTemp(int16_t* value);
    //Get manufacturer name
    bool getManufacturer(char* buf, int len);
    //Get device name
    bool getDeviceName(char* buf, int len);
    //Get device chemistry
    bool getChemistry(char* buf, int len);
    //Get serial number
    bool getSerial(int16_t* value);
    //Get average current
    bool getAvgCurrent(int16_t* value);
    //Get remaining capacity
    bool getRemCap(int16_t* value);
    //Get full capacity
    bool getFullCap(int16_t* value);
};

#endif
