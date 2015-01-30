#ifndef I2C_BUSES_H_
#define I2C_BUSES_H_

#include <avr/io.h>
#include <Arduino.h>

// constants for reading & writing
#define I2C_READ    1
#define I2C_WRITE   0

struct i2c_bus_t {
  boolean (*init)(void);
  bool (*start)(uint8_t);
  void (*start_wait)(uint8_t);
  bool (*rep_start)(uint8_t);
  bool (*write)(uint8_t);
  uint8_t (*read)(bool);
  void (*stop)(void);
};

extern i2c_bus_t i2c_bus0;
extern i2c_bus_t i2c_bus1;
extern i2c_bus_t i2c_bus2;
extern i2c_bus_t i2c_bus3;
extern i2c_bus_t i2c_bus4;
extern i2c_bus_t i2c_bus5;
extern i2c_bus_t i2c_bus6;
extern i2c_bus_t i2c_bus7;

#endif
