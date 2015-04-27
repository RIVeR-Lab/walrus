
#ifndef CONSTANTS_H
#define CONSTANTS_H

//Status blink rates
#define STATUS_OK 0x2AA
#define STATUS_NO_CONTROL_DATA 0x2A0
#define STATUS_NO_CONNECTION 0x280

//Status LED (pin sources LED current)
#define P_LED_STATUS 12

//External LED pins (for analogWrite)
#define P_EXT_LED_1 14 //MCU Pin: PC4; MCU Output Compare: OCR3C; Silkscreen: LED Board Control - 4 / J26; Schematic: LED1
#define P_EXT_LED_2 15 //MCU Pin: PC5; MCU Output Compare: OCR3B; Silkscreen: LED Board Control - 5 / J29; Schematic: LED2
#define P_EXT_LED_3 16 //MCU Pin: PB4; MCU Output Compare: OCR2A; Silkscreen: Pod Motor Control - 3 / J27; Schematic: POD_MOTOR1

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
#define CONTROL_DATA_TIMEOUT 1500

//OneWire commands
#define T_CONVERT 0x44
#define READ_SCRATCH 0xBE

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
#define INVALID_REQUEST_ERROR "Received invalid control message type."

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

