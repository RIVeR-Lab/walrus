
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
bool SmartBatt::getVoltage(int16_t* value)
{
    if (started)
         return read_word(i2c_bus, 11, 0x09, value);
    return -1;
}

bool SmartBatt::getCurrent(int16_t* value)
{
    if (started)
        return read_word(i2c_bus, 11, 0x0a, value);
    return -1;
}

//Get the battery output charge in hundredths of a percent (0.01%)
bool SmartBatt::getCharge(int16_t* value)
{
    if (started)
        return read_word(i2c_bus, 11, 0x0d, value);
    return -1;
}

//Get the battery temperature in hundredths of a degree C
bool SmartBatt::getTemp(int16_t* value)
{
    if (started)
        return read_word(i2c_bus, 11, 0x08, value);
    return -1;
}

//Get manufacturer name
bool SmartBatt::getManufacturer(char* buf, int size)
{
    if (started)
    {
         int len;
         len = read_block(i2c_bus, 11, 0x20, (byte*)buf, size-1);
         if (len == -1)
            strncpy(buf, "Failed to read manufacturer", size);
         else
         {
            buf[len] = '\0';   
            return true;  
         }               
    }
    return false;
}

//Get device name
bool SmartBatt::getDeviceName(char* buf, int size)
{
    if (started)
    {
         int len;
         len = read_block(i2c_bus, 11, 0x21, (byte*)buf, size-1);
         if (len == -1)
            strncpy(buf, "Failed to read device name", size);
         else
         {
            buf[len] = '\0';   
            return true;  
         }           
    }
    return false;
}

//Get device chemistry
bool SmartBatt::getChemistry(char* buf, int size)
{
    if (started)
    {
         int len;
         len = read_block(i2c_bus, 11, 0x22, (byte*)buf, size-1);
         if (len == -1)
            strncpy(buf, "Failed to read chemistry", size);
         else
         {
            buf[len] = '\0';   
            return true;  
         }      
    }
    return false;
}

//Get serial number
bool SmartBatt::getSerial(int16_t* value)
{
    if (started)
        return read_word(i2c_bus, 11, 0x1c, value);
    return -1;
}

//Get average current
bool SmartBatt::getAvgCurrent(int16_t* value)
{
    if (started)
         return read_word(i2c_bus, 11, 0x0b, value);
    return -1;
}

//Get remaining capacity
bool SmartBatt::getRemCap(int16_t* value)
{
    if (started)
        return read_word(i2c_bus, 11, 0x0f, value);
    return -1;
}

//Get full capacity
bool SmartBatt::getFullCap(int16_t* value)
{
    if (started)
        return read_word(i2c_bus, 11, 0x10, value);
    return -1;
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
        int pec = bus->read(true);
        ret = num;
      }
    }
    bus->stop();
    return ret;
}

int SmartBatt::read_word(i2c_bus_t* bus, byte addr, byte command_code, int16_t* value){
  int ret = -1;
  if (bus->start((addr << 1) | I2C_WRITE)) {
      bus->write(command_code);
      bus->stop();

      if(bus->rep_start((addr << 1) | I2C_READ)){
        int low = bus->read(false);
        int high = bus->read(false);
        int pec = bus->read(true);
        int val = (high & 0xFF) << 8 | low & 0xFF;
        (*value) = val;
        ret = 0;
      }
    }
    bus->stop();
    return ret;
}
