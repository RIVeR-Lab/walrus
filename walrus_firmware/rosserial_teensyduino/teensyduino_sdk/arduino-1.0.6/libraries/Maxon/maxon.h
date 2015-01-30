/*
This is an Arduino driver for the maxon motors 1-Q-EC Amplifier DEC Module 24/2.
Author: Brian Eccles
*/

#ifndef MAXON_H
#define MAXON_H

#include <Arduino.h>

//Definitions for setting the controller speed mode
//Motor type -  1 pole pair 	| 4 pole pair		| 8 pole pair
// 	------------------------------------------------------
//     OPEN - 	Open loop speed control 0-100%
//     SLOW - 	500-5,000 RPM	| 125-1,250 RPM 	| 62-625 RPM
//		MED -	500-20,000 RPM	| 125-5,000 RPM		| 62-2,500 RPM
//	   FAST -	500-80,000 RPM	| 125-20,000 RPM	| 62-10,000 RPM
#define SPEED_MODE_OPEN 0
#define SPEED_MODE_SLOW 2
#define SPEED_MODE_MED 1
#define SPEED_MODE_FAST 3

class Maxon
{
private
	//Speed mode pins
	int p_in1, p_int2
	//Direction control pin
	int p_dir;
	//Enable pin
	int p_en;
	//Speed control pin
	int p_spd;
	//Ready output pin
	int p_rdy;
	//Status led pin
	int p_led;
	//LED Direction (0 = sink, 1 = source)
	int led_dir;
	//True if begin has been called
	bool started;
	
public:
	Maxon();
	
	//Setup this object, must call before using other functions
	void begin(int in1, int in2, int dir, int en, int spd, int rdy, int led);
	
	//Sustain status led reading
	void sustain();
	
	//Drives the motor at the given speed
	void setMotor(int speed);
	
	//Enable or disable motor drive output
	void enable();
	void disable();
	
	//Sets the direction of the status lED feature (default is 1)
	void setLEDDir(int dir);
	
	//Set the speed mode (see definitions above)
	void setMode(int mode);
}

#endif