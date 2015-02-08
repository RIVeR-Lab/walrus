#include "i2c_buses.h"

#include <avr/io.h>
#include <Arduino.h>


#define SDA_PORT_0 PORTA
#define SDA_PIN_0 0
#define SCL_PORT_0 PORTA
#define SCL_PIN_0 1
#define I2C_TIMEOUT_0 500
#define I2C_NOINTERRUPT_0 0
#define I2C_SLOWMODE_0 1

#define SDA_PORT_1 PORTA
#define SDA_PIN_1 2
#define SCL_PORT_1 PORTA
#define SCL_PIN_1 3
#define I2C_TIMEOUT_1 500
#define I2C_NOINTERRUPT_1 0
#define I2C_SLOWMODE_1 1

#define SDA_PORT_2 PORTA
#define SDA_PIN_2 4
#define SCL_PORT_2 PORTA
#define SCL_PIN_2 5
#define I2C_TIMEOUT_2 500
#define I2C_NOINTERRUPT_2 0
#define I2C_SLOWMODE_2 1

#define SDA_PORT_3 PORTA
#define SDA_PIN_3 6
#define SCL_PORT_3 PORTA
#define SCL_PIN_3 7
#define I2C_TIMEOUT_3 500
#define I2C_NOINTERRUPT_3 0
#define I2C_SLOWMODE_3 1

#define SDA_PORT_4 PORTC
#define SDA_PIN_4 0
#define SCL_PORT_4 PORTC
#define SCL_PIN_4 1
#define I2C_TIMEOUT_4 500
#define I2C_NOINTERRUPT_4 0
#define I2C_SLOWMODE_4 1

#define SDA_PORT_5 PORTD
#define SDA_PIN_5 4
#define SCL_PORT_5 PORTD
#define SCL_PIN_5 5
#define I2C_TIMEOUT_5 500
#define I2C_NOINTERRUPT_5 0
#define I2C_SLOWMODE_5 1

#define SDA_PORT_6 PORTD
#define SDA_PIN_6 6
#define SCL_PORT_6 PORTD
#define SCL_PIN_6 7
#define I2C_TIMEOUT_6 500
#define I2C_NOINTERRUPT_6 0
#define I2C_SLOWMODE_6 1

#define SDA_PORT_7 PORTE
#define SDA_PIN_7 0
#define SCL_PORT_7 PORTE
#define SCL_PIN_7 1
#define I2C_TIMEOUT_7 500
#define I2C_NOINTERRUPT_7 0
#define I2C_SLOWMODE_7 1

#include <SoftI2CMaster0.h>
#include <SoftI2CMaster1.h>
#include <SoftI2CMaster2.h>
#include <SoftI2CMaster3.h>
#include <SoftI2CMaster4.h>
#include <SoftI2CMaster5.h>
#include <SoftI2CMaster6.h>
#include <SoftI2CMaster7.h>


i2c_bus_t i2c_bus0 = {i2c_init_0, i2c_start_0, i2c_start_wait_0, i2c_rep_start_0, i2c_write_0, i2c_read_0, i2c_stop_0};
i2c_bus_t i2c_bus1 = {i2c_init_1, i2c_start_1, i2c_start_wait_1, i2c_rep_start_1, i2c_write_1, i2c_read_1, i2c_stop_1};
i2c_bus_t i2c_bus2 = {i2c_init_2, i2c_start_2, i2c_start_wait_2, i2c_rep_start_2, i2c_write_2, i2c_read_2, i2c_stop_2};
i2c_bus_t i2c_bus3 = {i2c_init_3, i2c_start_3, i2c_start_wait_3, i2c_rep_start_3, i2c_write_3, i2c_read_3, i2c_stop_3};
i2c_bus_t i2c_bus4 = {i2c_init_4, i2c_start_4, i2c_start_wait_4, i2c_rep_start_4, i2c_write_4, i2c_read_4, i2c_stop_4};
i2c_bus_t i2c_bus5 = {i2c_init_5, i2c_start_5, i2c_start_wait_5, i2c_rep_start_5, i2c_write_5, i2c_read_5, i2c_stop_5};
i2c_bus_t i2c_bus6 = {i2c_init_6, i2c_start_6, i2c_start_wait_6, i2c_rep_start_6, i2c_write_6, i2c_read_6, i2c_stop_6};
i2c_bus_t i2c_bus7 = {i2c_init_7, i2c_start_7, i2c_start_wait_7, i2c_rep_start_7, i2c_write_7, i2c_read_7, i2c_stop_7};
