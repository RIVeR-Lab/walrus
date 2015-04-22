
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

//Status LED (pin sources LED current)
#define P_LED_STATUS 12

//SERVO MODE
//Motor control pins (for servo PWM)
#define P_MOTOR_1 25 //MCU Pin: PB5; MCU Output Compare: OCR1A; Silkscreen: Pod Motor Control - 1 / J28; Schematic: POD_MOTOR2
#define P_MOTOR_2 16 //MCU Pin: PC6; MCU Output Compare: OCR3A; Silkscreen: LED Board Control - 6 / J32; Schematic: LED3
#define P_MOTOR_3 27 //MCU Pin: PB7; MCU Output Compare: OCR1C; Silkscreen: Pod Motor Control - 2 / J31; Schematic: POD_MOTOR4
#define P_MOTOR_4 26 //MCU Pin: PB6; MCU Output Compare: OCR1B; Silkscreen: Pod Motor Control - 4 / J30; Schematic: POD_MOTOR3
//External LED pins (for analogWrite)
#define P_EXT_LED_1 14 //MCU Pin: PC4; MCU Output Compare: OCR3C; Silkscreen: LED Board Control - 4 / J26; Schematic: LED1
#define P_EXT_LED_2 15 //MCU Pin: PC5; MCU Output Compare: OCR3B; Silkscreen: LED Board Control - 5 / J29; Schematic: LED2
#define P_EXT_LED_3 24 //MCU Pin: PB4; MCU Output Compare: OCR2A; Silkscreen: Pod Motor Control - 3 / J27; Schematic: POD_MOTOR1

//SERIAL MODE
#define P_FRONT_TX 25
#define P_FRONT_RX 16
#define P_BACK_TX 27
#define P_BACK_RX 26
#define MAKE_MOTOR_1_BYTE(power) (((int16_t)power-1500)+64)
#define MAKE_MOTOR_2_BYTE(power) (((int16_t)power-1500)+192)
#define MAKE_MOTOR_3_BYTE(power) (((int16_t)power-1500)+64)
#define MAKE_MOTOR_4_BYTE(power) (((int16_t)power-1500)+192)

//TIMER MODE
//16 bit timer top value
#define TIMER_TOP 10000
//Motor control output compare registers
#define REG_MOTOR_1 OCR1A
#define REG_MOTOR_2 OCR3A
#define REG_MOTOR_3 OCR1C
#define REG_MOTOR_4 OCR1B
//LED output compare registers
#define SET_LED1(value) OCR3C = value*10
#define SET_LED2(value) OCR3B = value*10
#define SET_LED3(value) OCR2A = value/4


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

//Current Sensing
#define CURRENT_AVERAGE_SAMPLES 10
#define CURRENT_ADC_MAX 1023 
#define ADC_TO_mA(adcVal) (int16_t)((73300*(long)adcVal/CURRENT_ADC_MAX) - 36700) //See Pololu part #2453

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
#define NO_MOTOR_DATA_ERROR "Received no motor data when enabled, disabling..."

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

