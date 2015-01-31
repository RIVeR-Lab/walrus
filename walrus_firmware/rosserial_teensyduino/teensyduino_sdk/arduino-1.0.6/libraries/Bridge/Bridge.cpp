#include "Bridge.h"

//Constructor
Bridge::Bridge()
{
	started = false;
}

//Setup this object, must call before using other functions
void Bridge::begin(int in1, int in2, int in3, int in4, int d1, int d2, int d3, int d4, int sfa, int sfb, int ledA, int ledB)
{
	started = true;
	p_in1 = in1;
	p_in2 = in2;
	p_in3 = in3;
	p_in4 = in4;
	p_d1 = d1;
	p_d2 = d2;
	p_d3 = d4;
	p_d4 = d4;
	p_sfA = sfa;
	p_sfB = sfb;
	p_ledA = ledA;
	p_ledB = ledB;
	
	//Setup pins
	pinMode(p_in1, OUTPUT);
	pinMode(p_in2, OUTPUT);
	pinMode(p_in3, OUTPUT);
	pinMode(p_in4, OUTPUT);
	pinMode(p_d1, OUTPUT);
	pinMode(p_d2, OUTPUT);
	pinMode(p_d3, OUTPUT);
	pinMode(p_d4, OUTPUT);
	pinMode(p_sfA, INPUT);
	pinMode(p_sfB, INPUT);
	pinMode(p_ledA, OUTPUT);
	pinMode(p_ledB, OUTPUT);
	
	//Pull enable pins high (disable pins will be used to disable)
	digitalWrite(p_d2, HIGH);
	digitalWrite(p_d4, HIGH);
	
	//Set Defaults
	setCoast(CHAN_A);
	setCoast(CHAN_B);
	setLEDDir(1);
	setMotor(CHAN_A, 0);
	setMotor(CHAN_B, 0);
	disable(CHAN_A);
	disable(CHAN_B);
}

//Sustain status LEDs
void Bridge::sustain()
{
	if (started)
	{
		digitalWrite(p_ledA, !(digitalRead(p_sfA) ^ led_dir));
		digitalWrite(p_ledB, !(digitalRead(p_sfB) ^ led_dir));
	}
}

//Drives the given motor channel at the given speed
//channel - Motor channel (use definitions above)
//speed - Speed of motor (-255 to 255)
void Bridge::setMotor(int channel, int speed)
{
	if (started)
	{
		if (channel == CHAN_A)
		{
			if (speed > 0)
			{
				analogWrite(p_in1, speed);
				digitalWrite(p_in2, LOW);
				digitalWrite(p_d1, HIGH);
			}
			else if (speed < 0)
			{
				speed = speed*-1;
				analogWrite(p_in2, speed);
				digitalWrite(p_in1, LOW);
				digitalWrite(p_d1, HIGH);
			}
			else
			{
				digitalWrite(p_in1, LOW);
				digitalWrite(p_in2, LOW);
				if (coastA)
					digitalWrite(p_d1, LOW);
				else
					digitalWrite(p_d1, HIGH);
			}
				
		}
		else
		{
			if (speed > 0)
			{
				analogWrite(p_in3, speed);
				digitalWrite(p_in4, LOW);
				digitalWrite(p_d3, HIGH);
			}
			else if (speed < 0)
			{
				speed = speed*-1;
				analogWrite(p_in4, speed);
				digitalWrite(p_in3, LOW);
				digitalWrite(p_d3, HIGH);
			}
			else
			{
				digitalWrite(p_in3, LOW);
				digitalWrite(p_in4, LOW);
				if (coastA)
					digitalWrite(p_d3, LOW);
				else
					digitalWrite(p_d3, HIGH);
			}
				
		}
	}
}

//Enable motor output
void Bridge::enable(int channel)
{
	if (started)
	{
		if (channel == CHAN_A)
			digitalWrite(p_d2, HIGH);
		else
			digitalWrite(p_d4, HIGH);
	}
}

//Disable motor output
void Bridge::disable(int channel)
{
	if (started)
	{
		if (channel == CHAN_A)
			digitalWrite(p_d2, LOW);
		else
			digitalWrite(p_d4, LOW);
	}
}

//Sets the direction of the status led feature
void Bridge::setLEDDir(int dir)
{
	led_dir = dir;
}

//Set the provided channel to brake mode
void Bridge::setBrake(int channel)
{
	if (channel == CHAN_A)
		coastA = MOTOR_BRAKE;
	else
		coastB = MOTOR_BRAKE;
}

//Set the provided channel to coast mode
void Bridge::setCoast(int channel)
{
	if (channel == CHAN_A)
		coastA = MOTOR_COAST;
	else
		coastB = MOTOR_COAST;
}

//Set the stop mode of the provided channel to the given brake/coast setting
void Bridge::setStopMode(int channel, int brakecoast)
{
	if (channel == CHAN_A)
		coastA = brakecoast;
	else
		coastB = brakecoast;
}

