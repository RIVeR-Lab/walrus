#include <Arduino.h>

#include <avr/io.h>
#include "i2c_buses.h"

#include <ros.h>
#include <std_msgs/String.h>

//ros::NodeHandle nh;

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

void setup(void) {
  //nh.initNode();

  Serial.begin(19200); // change baudrate to 2400 on terminal when low CPU freq!
  Serial.println(F("Intializing ..."));
  if (!i2c_bus0.init())
    Serial.println(F("Initialization error on bus 0. SDA or SCL are low"));
  else
    Serial.println(F("Bus 0 Initialized"));
  if (!i2c_bus1.init())
    Serial.println(F("Initialization error on bus 1. SDA or SCL are low"));
  else
    Serial.println(F("Bus 1 Initialized"));
}

void loop(void)
{
  char buf[30];
  int len;

  Serial.print("Manufacture Name: ");
  Serial.print(len = read_block(&i2c_bus0, 11, 0x20, (byte*)buf, 30), DEC);
  buf[len] = '\0';
  Serial.println(buf);
  Serial.print("Manufacture Name: ");
  Serial.print(len = read_block(&i2c_bus1, 11, 0x20, (byte*)buf, 30), DEC);
  buf[len] = '\0';
  Serial.println(buf);
  delay(500);

  Serial.print("Device Name: ");
  Serial.print(len = read_block(&i2c_bus0, 11, 0x21, (byte*)buf, 30), DEC);
  buf[len] = '\0';
  Serial.println(buf);
  Serial.print("Device Name: ");
  Serial.print(len = read_block(&i2c_bus1, 11, 0x21, (byte*)buf, 30), DEC);
  buf[len] = '\0';
  Serial.println(buf);
  delay(500);

  Serial.print("Device Chemistry: ");
  Serial.print(len = read_block(&i2c_bus0, 11, 0x22, (byte*)buf, 30), DEC);
  buf[len] = '\0';
  Serial.println(buf);
  Serial.print("Device Chemistry: ");
  Serial.print(len = read_block(&i2c_bus1, 11, 0x22, (byte*)buf, 30), DEC);
  buf[len] = '\0';
  Serial.println(buf);
  delay(500);

  Serial.print("Serial Number: ");
  Serial.print(read_word(&i2c_bus0, 11, 0x1c), HEX);
  Serial.println();
  Serial.print("Serial Number: ");
  Serial.print(read_word(&i2c_bus1, 11, 0x1c), HEX);
  Serial.println();
  delay(500);

  Serial.print("Temperature: ");
  Serial.print(read_word(&i2c_bus0, 11, 0x08), DEC);
  Serial.println(" 1/10 K");
  Serial.print("Temperature: ");
  Serial.print(read_word(&i2c_bus1, 11, 0x08), DEC);
  Serial.println(" 1/10 K");
  delay(500);

  Serial.print("Battery Voltage: ");
  Serial.print(read_word(&i2c_bus0, 11, 0x09), DEC);
  Serial.println(" mV");
  Serial.print("Battery Voltage: ");
  Serial.print(read_word(&i2c_bus1, 11, 0x09), DEC);
  Serial.println(" mV");
  delay(500);

  Serial.print("Current: ");
  Serial.print(read_word(&i2c_bus0, 11, 0x0a), DEC);
  Serial.println(" mA");
  Serial.print("Current: ");
  Serial.print(read_word(&i2c_bus1, 11, 0x0a), DEC);
  Serial.println(" mA");
  delay(500);

  Serial.print("Average Current: ");
  Serial.print(read_word(&i2c_bus0, 11, 0x0b), DEC);
  Serial.println(" mA");
  Serial.print("Average Current: ");
  Serial.print(read_word(&i2c_bus1, 11, 0x0b), DEC);
  Serial.println(" mA");
  delay(500);

  Serial.print("State of Charge: ");
  Serial.print(read_word(&i2c_bus0, 11, 0x0e), DEC);
  Serial.println(" %");
  Serial.print("State of Charge: ");
  Serial.print(read_word(&i2c_bus1, 11, 0x0e), DEC);
  Serial.println(" %");
  delay(500);

  Serial.print("Remaining Capacity: ");
  Serial.print(read_word(&i2c_bus0, 11, 0x0f), DEC);
  Serial.println(" mAh");
  Serial.print("Remaining Capacity: ");
  Serial.print(read_word(&i2c_bus1, 11, 0x0f), DEC);
  Serial.println(" mAh");
  delay(500);

  Serial.print("Full Charge Capacity: ");
  Serial.print(read_word(&i2c_bus0, 11, 0x10), DEC);
  Serial.println(" mAh");
  Serial.print("Full Charge Capacity: ");
  Serial.print(read_word(&i2c_bus1, 11, 0x10), DEC);
  Serial.println(" mAh");

  Serial.println();

  //nh.spinOnce();

  delay(1000);
}
