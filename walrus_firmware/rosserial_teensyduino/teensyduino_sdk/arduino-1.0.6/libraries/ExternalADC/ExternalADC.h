/*
This is an Arduino driver for the Maxim MAX11612-17 family of analog to digital
converters. It utilizaes the standard Arduino Wire library for I2C communications.
Author: Brian Eccles
*/

#ifndef EXTERNALADC_H
#define EXTERNALADC_H

#include "Arduino.h"
#include <Wire.h>

#define SETUP_BYTE 0x82
#define CONFIGURATION_BYTE 0x01

class ExternalADC
{
    
private:
    //I2C Slave Address
    int address;
    //Number of channels to read on sustain
    int channels;
    //True if begin has been called
    bool started;
    //Array of ADC samples read in during sustain
    int samples[12];

public:
    ExternalADC();
    
    //Setup this object, must call before using sustain and get functions
    //address - Slave address of device (check datasheet)
    //channels - Number of channels to read from device (must be <= available channels)
    void begin(int address, int channels);
    
    //Sustains ADC sampling by receiving samples from the ADC
    void sustain();
    
    //Returns the sampled analog value on the provided channel (0-4095)
    int getValue(int channel);
    
};

#endif 
