
#include "SmartBatt.h"



//Constructor
SmartBatt::SmartBatt()
{
	started = false;
}
	
//Setup this object, must be called before using other functions
void SmartBatt::begin(i2c_bus_t* bus)
{
	started = true;
	i2c_bus = bus;
	bus->init();
}

//Get the battery output voltage in mV
int SmartBatt::getVoltage()
{
	if (started)
		 return read_word(i2c_bus, 11, 0x09);
	return 0;
}

//Get the battery output current in mA
int SmartBatt::getCurrent()
{
	if (started)
		return read_word(i2c_bus, 11, 0x0a);
	return 0;
}

//Get the battery output charge in hundredths of a percent (0.01%)
int SmartBatt::getCharge()
{
	if (started)
		return read_word(i2c_bus, 11, 0x0e);
	return 0;
}

//Get the battery temperature in hundredths of a degree C
int SmartBatt::getTemp()
{
	if (started)
		return read_word(i2c_bus, 11, 0x08);
	return 0;
}

//Get manufacturer name
void SmartBatt::getManufacturer(char* buf, int len)
{
	if (started)
	{
		 int len;
		 len = read_block(i2c_bus, 11, 0x20, (byte*)buf, len);
		 buf[len] = '\0';		 
	}
}

//Get device name
void SmartBatt::getDeviceName(char* buf, int len)
{
	if (started)
	{
		 int len;
		 len = read_block(i2c_bus, 11, 0x21, (byte*)buf, len);
		 buf[len] = '\0';		 
	}
}

//Get device chemistry
void SmartBatt::getChemistry(char* buf, int len)
{
	if (started)
	{
		 int len;
		 len = read_block(i2c_bus, 11, 0x22, (byte*)buf, len);
		 buf[len] = '\0';		 
	}
}

//Get serial number
int SmartBatt::getSerial()
{
	if (started)
		return read_word(i2c_bus, 11, 0x1c);
	return 0;
}

//Get average current
int SmartBatt::getAvgCurrent()
{
	if (started)
		 return read_word(i2c_bus, 11, 0x0b);
	return 0;
}

//Get remaining capacity
int SmartBatt::getRemCap()
{
	if (started)
		return read_word(i2c_bus, 11, 0x0f);
	return 0;
}

//Get full capacity
int SmartBatt::getFullCap()
{
	if (started)
		return read_word(i2c_bus, 11, 0x10);
	return 0;
}


//Private functions
int SmartBatt::read_block(i2c_bus_t* bus, byte addr, byte command_code, byte* buf, int buf_size){
  int ret = -1;
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
        ret = num;
      }
    }
    bus->stop();
    return ret;
}
int SmartBatt::read_word(i2c_bus_t* bus, byte addr, byte command_code){
  int ret = -1;
  if (bus->start((addr << 1) | I2C_WRITE)) {
      bus->write(command_code);
      bus->stop();

      if(bus->rep_start((addr << 1) | I2C_READ)){
        int low = bus->read(false);
        int high = bus->read(false);
        int pec = bus->read(false);
        int val = (high & 0xFF) << 8 | low & 0xFF;
        ret = val;
      }
    }
    bus->stop();
    return ret;
}
