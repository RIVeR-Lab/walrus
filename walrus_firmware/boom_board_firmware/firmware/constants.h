
#ifndef CONSTANTS_H
#define CONSTANTS_H

//Status LED (pin sources LED Current)
#define P_LED_STATUS 18 //PE6

//Maxon motor controller pins
#define P_MAXON_DIR 28 //PA0
#define P_MAXON_EN 29 //PA1
#define P_MAXON_IN1 30 //PA2
#define P_MAXON_IN2 31 //PA3
#define P_MAXON_READY 32 //PA4
#define P_MAXON_SPEED 16 //PC6
#define P_MAXON_STATUS 46 //PE3

//H-Bridge pins
#define P_BRIDGE_IN1 14 //PC4
#define P_BRIDGE_IN2 25 //PB5
#define P_BRIDGE_IN3 26 //PB6
#define P_BRIDGE_IN4 27 //PB7
#define P_BRIDGE_D1 2 //PD2
#define P_BRIDGE_D2 3 //PD3
#define P_BRIDGE_D3 4 //PD4
#define P_BRIDGE_D4 5 //PD5
#define P_BRIDGE_SFA 10 //PC0
#define P_BRIDGE_SFB 11 //PC1
#define P_BRIDGE_LEDA 36 //PE4
#define P_BRIDGE_LEDB 37 //PE5

//Analog Expansion pins 
#define P_ANALOG_EXP_1 0 //J4/J5
#define P_ANALOG_EXP_2 1 //J6/J7
#define P_ANALOG_EXP_3 2 //J8/J9
#define P_ANALOG_EXP_4 3 //J10/J11

//LPF Filter
#define P_LPF_CLK //PB4

//Camera LED pin
#define P_CAM_LED //PC5

//External ADC Channels
#define CHAN_CO_SENSE 0
#define CHAN_LPG_SENSE 1
#define CHAN_CNG_SENSE 2
#define CHAN_H_SENSE 3
#define CHAN_PAN_CURRENT 4
#define CHAN_TILT_CURRENT 5
#define CHAN_PAN_POT 5
#define CHAN_TILT_POT 6
#define CHAN_DEPLOY_POT 7

//I2C Slave addresses
#define ADDR_EXT_ADC 0x35
#define ADDR_TEMP_HUMID 0x40

//H-Bridge Motor Channels
#define CHAN_PAN_MOTOR 0
#define CHAN_TILT_MOTOR 1

//Timing
#define ROS_MSG_RATE 10
#define MOTOR_OFF_TIMOUT 500

//Status LED blink rates
#define STATUS_OK 100
#define STATUS_NO_PC 1000
#define STATUS_NO_MSG 250


#endif