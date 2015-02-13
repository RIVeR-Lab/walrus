
#include "TempHumid.h"

//Constructor
TempHumid::TempHumid()
{
	started = false;
}

//Setup object and write configuration bytes to the sensor
void TempHumid::begin(int address)
{
	started = true;
	this->address = address;
	Wire.begin();
	Wire.beginTransmission(address);
	Wire.write(WRITE_REG_CMD);
	Wire.write(CONFIG_BYTE);
	Wire.endTransmission();
}

//Get the current temperature from the sensor
int TempHumid::getTemp()
{
	if (started)
	{
		Wire.beginTransmission(address);
		Wire.write(MEASURE_TEMP_CMD);
		Wire.endTransmission();
		Wire.requestFrom(address, 2);
		int timeout = millis();
		while (Wire.available() < 2)
		{
			if (millis() > timeout + 500)
				return 0;
		}
		int MSB = Wire.read();
		int LSB = Wire.read();
		long value = (MSB << 8) | LSB;
		return (int)((17572*value/65536)-46.85);
	}
	return 0;
}

//Get the current humidity from the sensor
int TempHumid::getHumidity()
{
	if (started)
	{
		Wire.beginTransmission(address);
		Wire.write(MEASURE_HUMID_CMD);
		Wire.endTransmission();
		Wire.requestFrom(address, 2);
		int timeout = millis();
		while (Wire.available() < 2)
		{
			if (millis() > timeout + 500)
				return 0;
		}
		int MSB = Wire.read();
		int LSB = Wire.read();
		long value = (MSB << 8) | LSB;
		return (int)((12500*value/65536)-600);
	}
	return 0;
} 