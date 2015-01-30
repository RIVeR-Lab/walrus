
#include "ExternalADC.h"

//Constructor
ExternalADC::ExternalADC()
{
	started = false;
}

//Setup object and write configuration bytes to the sensor
void ExternalADC::begin(int address, int channels)
{
	Wire.begin();
	Wire.beginTransmission(address);
	Wire.write(SETUP_BYTE);
	Wire.write(CONFIGURATION_BYTE | ((channels+1) << 1));
	Wire.endTransmission();
	this->address = address;
	this->channels = channels;
	started = true;
}

//Sustains the ADC operation by receiving samples for all channels
void ExternalADC::sustain()
{
	Wire.requestFrom(address, 2*channels);
	int LSB, MSB;
	for (int l = 0; l < channels; l++)
	{
		MSB = Wire.read();
		LSB = Wire.read();
		samples[l] = (MSB << 8) | LSB;
	}
}

//Returns the sample value of a given channel (0-4095)
int ExternalADC::getValue(int channel)
{
	if (channel >= 0 && channel < 12)
		return samples[channel];
	return 0;
}

