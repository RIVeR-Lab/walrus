
#ifndef CONSTANTS_H
#define CONSTANTS_H

//Status LED (pin sources LED current)
#define P_LED_STATUS 12

//Motor control pins (servo PWM)
#define P_MOTOR_1 25
#define P_MOTOR_2 27
#define P_MOTOR_3 24
#define P_MOTOR_4 26

//Current sensor pins (analog)
#define P_CURRENT_1 4
#define P_CURRENT_2 5
#define P_CURRENT_3 6
#define P_CURRENT_4 7

//Encoder pins (analog)
#define P_ENCODER_1 0
#define P_ENCODER_2 1
#define P_ENCODER_3 2
#define P_ENCODER_4 3 

//Water sensor pins
#define P_WATER_1 37 //PE5
#define P_WATER_2 18 //PE6
#define P_WATER_3 19 //PE7
#define P_WATER_4 2 //PD2
#define P_WATER_5 3 //PD3
#define P_WATER_6 4 //PE4

//Temp sensor
#define P_EXT_TEMP 46 //PE3

//Contactor shut-off 
#define P_CONTACTOR 0 //PC7

//Battery SMBus lines
//Batter 1
#define P_BATT1UPR_SDA 28 //PA0
#define P_BATT1UPR_SCL 29 //PA1
#define P_BATT1LWR_SDA 30 //PA2
#define P_BATT1LWR_SCL 31 //PA3
//Battery 2
#define P_BATT2UPR_SDA 32 //PA4
#define P_BATT2UPR_SCL 33 //PA5
#define P_BATT2LWR_SDA 34 //PA6
#define P_BATT2LWR_SCL 35 //PA7
//Battery 3
#define P_BATT3UPR_SDA 10 //PC0
#define P_BATT3UPR_SCL 11 //PC1
#define P_BATT3LWR_SDA 4 //PD4
#define P_BATT3LWR_SCL 5 //PD5
//Battery 4
#define P_BATT4UPR_SDA 6 //PD6
#define P_BATT4UPR_SCL 7 //PD7
#define P_BATT4LWR_SDA 8 //PE0
#define P_BATT4LWR_SCL 9 //PE1

//External LED pins
#define P_EXT_LED_1 13 //PC4
#define P_EXT_LED_2 15 //PC5
#define P_EXT_LED_3 16 //PC6

//Tension Potentiometer Channels
#define CHAN_POT1 1
#define CHAN_POT2 0

//I2C Slave addresses
#define ADDR_TEMP_HUMID 0x40
#define ADDR_EXT_ADC 0x34

//Timing
#define ROS_MSG_RATE 10
#define TEMP_READ_RATE 2000
#define MOTOR_OFF_TIMOUT 500

//Status LED blink rates
#define STATUS_OK 100
#define STATUS_NO_PC 1000
#define STATUS_NO_MSG 250

//OneWire commands
#define T_CONVERT 0x44
#define READ_SCRATCH 0xBE

//Temperature sensor addresses
#define TEMP_1_EN true
#define TEMP_1_ADDR {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}
#define TEMP_2_EN true
#define TEMP_2_ADDR {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}
#define TEMP_3_EN true
#define TEMP_3_ADDR {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}
#define TEMP_4_EN true
#define TEMP_4_ADDR {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}
#define TEMP_5_EN true
#define TEMP_5_ADDR {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}
#define TEMP_6_EN true
#define TEMP_6_ADDR {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}
#define TEMP_7_EN true
#define TEMP_7_ADDR {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}
#define TEMP_8_EN true
#define TEMP_8_ADDR {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}
#define TEMP_9_EN true
#define TEMP_9_ADDR {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}
#define TEMP_10_EN true
#define TEMP_10_ADDR {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}



#endif

