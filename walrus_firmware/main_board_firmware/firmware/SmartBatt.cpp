
#include "SmartBatt.h"



//Constructor
SmartBatt()
{
	started = false;
}
	
//Setup this object, must be called before using other functions
void begin(i2c_bus_t bus)
{
	started = true;
	i2c_bus = bus;
}

//Get the battery output voltage in mV
int getVoltage()
{
	if (started)
		 return read_word(&i2c_bus, 11, 0x09);
	return 0;
}

//Get the battery output current in mA
int getCurrent()
{
	if (started)
		return read_word(&i2c_bus, 11, 0x0a)
	return 0;
}

//Get the battery output charge in hundredths of a percent (0.01%)
int getCharge()
{
	if (started)
		return read_word(&i2c_bus, 11, 0x0e);
	return 0;
}

//Get the battery temperature in hundredths of a degree C
int getTemp()
{
	if (started)
		return read_word(&i2c_bus, 11, 0x08);
	return 0;
}

//Returns true if the battery is in shutdown mode
bool getShutdown()
{
	if (started)
		 return 0;
	return 0;
}

//Get manufacturer name
char* getManufacturer()
{
	if (started)
	{
		 char buf[30];
		 int len;
		 len = read_block(&i2c_bus, 11, 0x20, (byte*)buf, 30)
		 buf[len] = '\0';		 
		 return buf;
	}
	return "";
}

//Get device name
char* getDeviceName()
{
	if (started)
	{
		 char buf[30];
		 int len;
		 len = read_block(&i2c_bus, 11, 0x21, (byte*)buf, 30)
		 buf[len] = '\0';		 
		 return buf;
	}
	return "";
}

//Get device chemistry
char* getChemistry()
{
	if (started)
	{
		 char buf[30];
		 int len;
		 len = read_block(&i2c_bus, 11, 0x22, (byte*)buf, 30)
		 buf[len] = '\0';		 
		 return buf;
	}
	return "";
}

//Get serial number
int getSerial()
{
	if (started)
		return read_word(&i2c_bus, 11, 0x1c);
	return "";
}

//Get average current
int getAvgCurrent()
{
	if (started)
		 return read_word(&i2c_bus, 11, 0x0b);
	return 0;
}

//Get remaining capacity
int getRemCap()
{
	if (started)
		return read_word(&i2c_bus, 11, 0x0f);
	return 0;
}

//Get full capacity
int getFullCap()
{
	if (started)
		return read_word(&i2c_bus, 11, 0x10);
	return 0;
}


//Private functions
int read_block(i2c_bus_t* bus, byte addr, byte command_code, byte* buf, int buf_size){
  if (bus->start((addr << 1) | I2C_WRITE)) {
      bus->write(command_code);
      bus->stop();

      if(bus->rep_start((addr << 1) | I2C_READ)){
        int num = bus->read(false);
        for(int i = 0; i < num; ++i) {
          int val = bus->read(false);
          if(i < buf_size)
            buf[i] = val;
        }
        int pec = bus->read(false);
        bus->stop();
        return num;
      }
      else
        return -1;
    }
    else
      return -1;
}
int read_word(i2c_bus_t* bus, byte addr, byte command_code){
  if (bus->start((addr << 1) | I2C_WRITE)) {
      bus->write(command_code);
      bus->stop();

      if(bus->rep_start((addr << 1) | I2C_READ)){
        int low = bus->read(false);
        int high = bus->read(false);
        int pec = bus->read(false);
        bus->stop();
        int val = (high & 0xFF) << 8 | low & 0xFF;
        return val;
      }
      else
        return -1;
    }
    else
      return -1;
}