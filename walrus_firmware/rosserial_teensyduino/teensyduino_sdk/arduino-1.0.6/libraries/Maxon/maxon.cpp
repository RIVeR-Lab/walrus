#include "maxon.h"

//Constructor
Maxon::Maxon()
{
	started = false;
}

//Setup this object, must call before using other fuctions
void Maxon::begin(int in1, int in2, int dir, int en, int spd, int rdy, int led)
{
	started = true;
	p_in1 = in1;
	p_in2 = in2;
	p_dir = dir;
	p_en = en;
	p_spd = spd;
	p_rdy = rdy;
	p_led = led;
	
	//Setup pins
	pinMode(p_in1, OUTPUT);
	pinMode(p_in2, OUTPUT);
	pinMode(p_dir, OUTPUT);
	pinMode(p_en, OUTPUT);
	pinMode(p_spd, OUTPUT);
	pinMode(p_rdy, INPUT);
	pinMode(p_led, OUTPUT);
	
	//Set defaults
	setMode(SPEED_MODE_OPEN);
	setLEDDir(1);
	disable();
	setMotor(0);
}

//Sustain status led reading
void Maxon::sustain()
{
	if (started)
		digitalWrite(p_led, !(digitalRead(p_rdy) ^ led_dir))
}

//Drives the motor at the given speed
void setMotor(int speed)
{
	if (started)
	{
		if (speed > 0)
		{
			digitalWrite(dir, HIGH);
			analogWrite(p_spd, speed);
		}
		else if (speed < 0)
		{
			speed = speed*-1;
			digitalWrite(dir, LOW);
			analogWrite(p_spd, speed);
		}
		else
			digitalWrite(p_spd, 0);
	}
}

//Enable the motor drive output
void enable()
{
	if (started)
		digitalWrite(en, HIGH);
}

//Disable the motor drive output
void disable()
{
	if (started)
		digitalWrite(en, LOW);
}

//Sets the direction of the status LED feature (default is 1)
void setLEDDir(int dir)
{
	led_dir = dir;
}

//Set the speed mode (see definitions in header file)
void setMode(int mode)
{
	if (started)
	{
		digitalWrite(p_in1, mode & 1);
		digitalWrite(p_in2, mode & 2);
	}
}
	
	