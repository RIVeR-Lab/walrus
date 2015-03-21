
#ifndef CONSTANTS_H
#define CONSTANTS_H

//ROS Diagnostics states
#define STATE_OK 0
#define STATE_WARN 1
#define STATE_ERROR 2
#define STATE_STALE 3

//Status structure
typedef struct Status {
    char* str;
    int16_t code;
    uint16_t blink_code;
} Status;

//Defined statuses
Status STATUS_ENABLED = {"Enabled", STATE_OK, /*10_1010_1010*/0x2AA}; 
Status STATUS_NO_HS_DATA = {"No motor data", STATE_ERROR, /*10_1010_1000*/0x2A8};
Status STATUS_DISABLED = {"Disabled", STATE_OK, /*11_1110_0000*/0x3E0};
Status STATUS_NO_CONTROL_DATA = {"No data", STATE_WARN, /*10_1010_0000*/0x2A0};
Status STATUS_NO_CONNECTION = {"No connection", STATE_ERROR, /*10_1000_0000*/0x280};

//Status LED (pin sources LED current)
#define P_LED_STATUS 12

//Motor control pins (servo PWM)
#define P_MOTOR_1 25
#define P_MOTOR_2 27
#define P_MOTOR_3 24
#define P_MOTOR_4 26

//Motor control registers when using timer1 for PWM
#define REG_MOTOR_1 OCR1A
#define REG_MOTOR_4 OCR1B
#define REG_MOTOR_2 OCR1C

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
#define P_WATER_6 36 //PE4

//Temp sensor
#define P_EXT_TEMP 46 //PE3

//Contactor shut-off 
#define P_CONTACTOR 17 //PC7

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
#define P_EXT_LED_1 14 //PC4
#define P_EXT_LED_2 15 //PC5
#define P_EXT_LED_3 16 //PC6

//Tension Potentiometer Channels
#define CHAN_POT1 1
#define CHAN_POT2 0

//I2C Slave addresses
#define ADDR_TEMP_HUMID 0x40
#define ADDR_EXT_ADC 0x34

//Timing
#define ROS_MSG_RATE 20
#define STATUS_BLINK_PERIOD 100
#define STATUS_BLINK_LENGTH 10
#define LED_TRIGGER(count) (count % (STATUS_BLINK_PERIOD/ROS_MSG_RATE) == 0)
#define LED_SEGMENT(count) ((count % (STATUS_BLINK_PERIOD/ROS_MSG_RATE*STATUS_BLINK_LENGTH))/(STATUS_BLINK_PERIOD/ROS_MSG_RATE))
#define HS_DATA_TIMEOUT 250
#define CONTROL_DATA_TIMEOUT 1500

//OneWire commands
#define T_CONVERT 0x44
#define READ_SCRATCH 0xBE

//Equations
#define CURRENT_ADC_MAX 1023 
#define ADC_TO_mA(adcVal) ((73300*adcVal/CURRENT_ADC_MAX) - 36700) //See Pololu part #2453

//Temperature sensor addresses
#define TEMP_1_ADDR {0x10, 0x9D, 0x92, 0xE2, 0x02, 0x08, 0x00, 0x43}
#define TEMP_2_ADDR {0x10, 0x35, 0x3F, 0xE2, 0x02, 0x08, 0x00, 0x61}
#define TEMP_3_ADDR {0x10, 0x92, 0x79, 0xE2, 0x02, 0x08, 0x00, 0xCD}
#define TEMP_4_ADDR {0x10, 0x81, 0x88, 0xE2, 0x02, 0x08, 0x00, 0xA4}
#define TEMP_5_ADDR {0x10, 0x1B, 0xEA, 0xE1, 0x02, 0x08, 0x00, 0xC0}
#define TEMP_6_ADDR {0x10, 0x22, 0x62, 0xE2, 0x02, 0x08, 0x00, 0xC6}
#define TEMP_7_ADDR {0x10, 0x14, 0x3E, 0xE3, 0x02, 0x08, 0x00, 0xA2}
#define TEMP_8_ADDR {0x10, 0xFF, 0x59, 0xE2, 0x02, 0x08, 0x00, 0xBC}
#define TEMP_9_ADDR {0x10, 0x66, 0xF4, 0xE1, 0x02, 0x08, 0x00, 0xCC}
#define TEMP_10_ADDR {0x10, 0x0B, 0x7F, 0xE2, 0x02, 0x08, 0x00, 0x76}

//Error strings
#define LOOP_TOO_LONG_ERROR "Main loop took longer than 20 ms."
#define SLOW_REQUEST_ERROR "Cannot complete request when enabled."
#define INVALID_REQUEST_ERROR "Received invalid control message type."
#define NO_MOTOR_DATA_ERROR "Enabled, but receiving no motor data."

//Data report states for low speed data aquisition state machine
enum LowSpeedOpState { READ_TEMPHUMID, 
                        READ_PRESSURE, 
                        READ_WATER, 
                        READ_EXTTEMP_1,
                        READ_EXTTEMP_2,
                        READ_EXTTEMP_3, 
                        READ_EXTTEMP_4, 
                        READ_EXTTEMP_5, 
                        READ_EXTTEMP_6, 
                        READ_EXTTEMP_7, 
                        READ_EXTTEMP_8, 
                        READ_EXTTEMP_9, 
                        READ_EXTTEMP_10, 
                        ISSUE_EXTTEMP_MEASURE, 
                        READ_TENSION, 
                        READ_BATT1_VOLTAGE, 
                        READ_BATT2_VOLTAGE,
                        READ_BATT3_VOLTAGE, 
                        READ_BATT4_VOLTAGE, 
                        READ_BATT1_CURRENT, 
                        READ_BATT2_CURRENT, 
                        READ_BATT3_CURRENT, 
                        READ_BATT4_CURRENT,       
                        READ_BATT1_AVGCURRENT, 
                        READ_BATT2_AVGCURRENT, 
                        READ_BATT3_AVGCURRENT, 
                        READ_BATT4_AVGCURRENT,     
                        READ_BATT1_TEMP, 
                        READ_BATT2_TEMP, 
                        READ_BATT3_TEMP, 
                        READ_BATT4_TEMP,    
                        READ_BATT1_CHARGE, 
                        READ_BATT2_CHARGE, 
                        READ_BATT3_CHARGE, 
                        READ_BATT4_CHARGE,
                        SEND_DATA,
                        LOW_SPEED_OP_STATE_NUM }; 


#endif

