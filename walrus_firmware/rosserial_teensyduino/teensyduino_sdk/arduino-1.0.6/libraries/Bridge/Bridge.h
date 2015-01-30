/*
This is an Arduino driver for the Freescale Semiconductor MC3392EK H-Bridge.
Author: Brian Eccles
*/

#ifndef BRIDGE_H
#define BRIDGE_H

#include <Arduino.h>

#define CHAN_A 0
#define CHAN_B 1

class Bridge
{
private:
	//Input pins
	int p_in1, p_in2, p_in3, p_in4;
	//Enable/Disable pins
	int p_d1, p_d2, p_d3, p_d4;
	//Status flag pins
	int p_sfA, p_sfB;
	//Status LED pins
	int p_ledA, p_ledB;
	//LED Direction (0 = sink, 1 = source;
	int led_dir;
	//BrakeCoast (0 = break, 1 = coast)
	int coastA, coastB;
	//True if begin has been called
	bool started;
	
public:
	Bridge();
	
	//Setup this object, must be called before using other functions
	void begin(int in1, int in2, int in3, int in4, int d1, int d2, int d3, int d4, int sfa, int sfb, int ledA, int ledB);
	
	//Sustain status LEDs
	void sustain();
	
	//Drives the given motor channel at the given speed
	//channel - Motor channel (use definitions above)
	//speed - Speed of motor (-255 to 255)
	void setMotor(int channel, int speed);
	
	//Enable or disable motor output
	void enable(int channel);
	void disable(int channel);

	//Sets the direction of the status led feature
	void setLEDDir(int dir);
	
	//Set the provided channel to brake or coast mode 
	void setBrake(int channel);
	void setCoast(int channel);
};


#endif
