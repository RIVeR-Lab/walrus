
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

//Issue a command to measure humidity and temperature, follow by a call to readAll at least 20ms later
void TempHumid::measure()
{
    if (started)
    {
        Wire.beginTransmission(address);
        Wire.write(MEASURE_HUMID_CMD_NOHOLD);
        Wire.endTransmission(false);
    }
}

//Reads humidity and temperature from the device, measure must be called about 20ms prior to calling this
//Returns true if reading succeeded
bool TempHumid::readAll()
{
    if (started)
    {
        bool success = Wire.requestFrom(address, 2) == 2;
        if (!success)
            return false;
        int MSB = Wire.read();
		int LSB = Wire.read();
		uint16_t value = (MSB << 8) | LSB;
		humidity = (int)((12500*(long)value/65536)-600);
		Wire.beginTransmission(address);
		Wire.write(READ_TEMP);
		Wire.endTransmission(false);
		Wire.requestFrom(address, 2);
		MSB = Wire.read();
		LSB = Wire.read();
		value = (MSB << 8) | LSB;
		temp =(int)((17572*(long)value/65536)-4685);
		return true;
    }
    return false;
}

//Get the temperature value read by readAll()
int TempHumid::getTemp()
{
    if (started)
        return temp;
    return 0;
}

//Get the humidity value read by readAll()
int TempHumid::getHumidity()
{
    if (started)
        return humidity;
    return 0;
}

//Get the current temperature from the sensor
int TempHumid::getTempNow()
{
	if (started)                  
	{
		Wire.beginTransmission(address);
		Wire.write(MEASURE_TEMP_CMD_NOHOLD);
		Wire.endTransmission();
		Wire.requestFrom(address, 2);
		int MSB = Wire.read();
		int LSB = Wire.read();
		uint16_t value = (MSB << 8) | LSB;
		return (int)((17572*(long)value/65536)-4685);
	}
	return 0;
}

//Get the current humidity from the sensor
int TempHumid::getHumidityNow()
{
	if (started)
	{
		Wire.beginTransmission(address);
		Wire.write(MEASURE_HUMID_CMD_NOHOLD);
		Wire.endTransmission();
		Wire.requestFrom(address, 2);
		int MSB = Wire.read();
		int LSB = Wire.read();
		uint16_t value = (MSB << 8) | LSB;
		return (int)((12500*(long)value/65536)-600);
	}
	return 0;
} 
