
#ifndef CONSTANTS_H
#define CONSTANTS_H

//ROS Diagnostics states
#define STATE_OK 0
#define STATE_WARN 1
#define STATE_ERROR 2
#define STATE_STALE 3

//Status structure
typedef struct Status {
    const char* str;
    int16_t code;
    uint16_t blink_code;
} Status;

//Defined statuses
Status STATUS_ENABLED = {"Enabled", STATE_OK, /*10_1010_1010*/0x2AA}; 
Status STATUS_DISABLED = {"Disabled", STATE_OK, /*11_1110_0000*/0x3E0};
Status STATUS_NO_CONTROL_DATA = {"No data", STATE_WARN, /*10_1010_0000*/0x2A0};
Status STATUS_NO_CONNECTION = {"No connection", STATE_ERROR, /*10_1000_0000*/0x280};

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
#define P_LPF_CLK 24//PB4
#define LPF_VALUE 5000

//Camera LED pin
#define P_CAM_LED 15//PC5

//External ADC Channels
#define NUM_CHANNELS 9
#define CHAN_CO_SENSE 0
#define CHAN_LPG_SENSE 1
#define CHAN_CNG_SENSE 2
#define CHAN_H_SENSE 3
#define CHAN_PAN_CURRENT 4
#define CHAN_TILT_CURRENT 5
#define CHAN_PAN_POT 6
#define CHAN_TILT_POT 7
#define CHAN_DEPLOY_POT 8

//I2C Slave addresses
#define ADDR_EXT_ADC 0x35
#define ADDR_TEMP_HUMID 0x40

//H-Bridge Motor Channels
#define CHAN_PAN_MOTOR 0
#define CHAN_TILT_MOTOR 1

//Current sensing
#define CURRENT_SENSE_R 270
#define CURRENT_SENSE_ 0.24
//0.24% of load current output from pins throught 270 ohm resistor
//V = I*R = (0.0024*I)*270 = 0.648*I = V; I = 1.543*V
#define ADC_TO_mA(adcVal) (adcVal*15)/10

//Timing
#define ROS_MSG_RATE 20
#define STATUS_BLINK_PERIOD 100
#define STATUS_BLINK_LENGTH 10
#define LED_TRIGGER(count) (count % (STATUS_BLINK_PERIOD/ROS_MSG_RATE) == 0)
#define LED_SEGMENT(count) ((count % (STATUS_BLINK_PERIOD/ROS_MSG_RATE*STATUS_BLINK_LENGTH))/(STATUS_BLINK_PERIOD/ROS_MSG_RATE))
#define HS_DATA_TIMEOUT 250
#define CONTROL_DATA_TIMEOUT 1500

//Error strings
#define LOOP_TOO_LONG_ERROR "Main loop took longer than 20 ms."
#define SLOW_REQUEST_ERROR "Cannot complete request when enabled."
#define INVALID_REQUEST_ERROR "Received invalid control message type."
#define NO_MOTOR_DATA_ERROR "Received no motor data when enabled, disabling..."

//Data report states for low speed data aquisition state machine
enum LowSpeedOpState { READ_TEMPHUMID, 
                        READ_GAS,
                        READ_ANALOG,
                        SEND_DATA,
                        LOW_SPEED_OP_STATE_NUM }; 


#endif
