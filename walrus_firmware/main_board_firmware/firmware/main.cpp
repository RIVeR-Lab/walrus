
//#define USE_SERVO
#define USE_SERIAL

#include <Arduino.h>
#include <ros.h>
#include <walrus_firmware_msgs/MainBoardHighSpeedControl.h>
#include <walrus_firmware_msgs/MainBoardHighSpeedFeedback.h>
#include <walrus_firmware_msgs/MainBoardLowSpeedData.h>
#include <walrus_firmware_msgs/MainBoardControl.h>
#include <Wire.h>
#include <TempHumid.h>
#include <MPL3115A2.h>
#undef STATUS //Defined in MPL3115A2.h
#include <ExternalADC.h>
#include <OneWire.h>
#ifdef USE_SERVO
    #include <Servo.h>
#elif defined USE_SERIAL
	#include <SoftwareSerial.h>
#endif
#include <SmartBatt.h>
#include "constants.h"


using namespace walrus_firmware_msgs;

//ROS node handle
ros::NodeHandle nh;

//Functions and messages for high speed motor control and feed back
bool output_disable = true;
bool hs_comm_valid = false;
long last_hs_msg = 0;
void doHighSpeedOperations();
void recv_hs_control(const walrus_firmware_msgs::MainBoardHighSpeedControl& msg);
walrus_firmware_msgs::MainBoardHighSpeedFeedback hs_feedback_msg;
walrus_firmware_msgs::MainBoardHighSpeedControl hs_control_msg;
ros::Publisher hs_feedback("main_board/hs_feedback", &hs_feedback_msg);
ros::Subscriber<walrus_firmware_msgs::MainBoardHighSpeedControl> hs_control("main_board/hs_control", &recv_hs_control);

//Do low speed motor control operations, stagger across cycles
void doLowSpeedOperations();
int report_state = 0;
walrus_firmware_msgs::MainBoardLowSpeedData ls_data_msg;
ros::Publisher ls_data("main_board/ls_data", &ls_data_msg);

//Control pipes
bool mark_error = false;
const char* err_str = "";
long last_control_msg = 0;
bool delayed_operation = false;
bool no_ls = false;
void recv_control(const walrus_firmware_msgs::MainBoardControl& msg);
walrus_firmware_msgs::MainBoardControl to_pc_data;
ros::Publisher to_pc("main_board/board_to_PC_control", &to_pc_data);
ros::Subscriber<walrus_firmware_msgs::MainBoardControl> from_pc("main_board/PC_to_board_control", &recv_control);


//Pressure sensor
MPL3115A2 pressure_sense;
//External ADC
ExternalADC extADC;
//Temperature and humidity sensor
TempHumid temphumid_sense;
//OneWire object for temperature sensors
OneWire exttemp_sense(P_EXT_TEMP);
//Motor servos
#ifdef USE_SERVO
    Servo motor1, motor2, motor3, motor4;
#elif defined USE_SERIAL
	SoftwareSerial front_mc(P_FRONT_RX, P_FRONT_TX, false);
	SoftwareSerial back_mc(P_BACK_RX, P_BACK_TX, false);
#endif
//Battery SMBus objects
SmartBatt upper[4], lower[4];
//Current sample average
int16_t current_samples [4][CURRENT_AVERAGE_SAMPLES];
uint8_t current_sample_ptr[4];
//LED pin array
uint8_t current_pins[] = {P_CURRENT_1, P_CURRENT_2, P_CURRENT_3, P_CURRENT_4};
uint8_t encoder_pins[] = {P_ENCODER_1, P_ENCODER_2, P_ENCODER_3, P_ENCODER_4};
uint8_t led_pins[] = {P_EXT_LED_1, P_EXT_LED_2, P_EXT_LED_3};
//Temp sensor addresses
uint8_t temp_addr[10][8] = {TEMP_1_ADDR, TEMP_2_ADDR, TEMP_3_ADDR, TEMP_4_ADDR, TEMP_5_ADDR, TEMP_6_ADDR, TEMP_7_ADDR, TEMP_8_ADDR, TEMP_9_ADDR, TEMP_10_ADDR};
//Temp sensor buffer;
uint8_t temp_buff[9];
//Count the number of loops for timing
long counter = 0;
//Status of system (to show on LED)
Status* status = &STATUS_ENABLED;
Status* last_status = &STATUS_ENABLED;
bool led_state = false;


//Functions for high speed motor control operations
void doHighSpeedOperations()
{
    //Write motor power data if enabled
    if (output_disable)
    {
        #ifdef USE_SERVO
            motor1.writeMicroseconds(1500);
            motor2.writeMicroseconds(1500);
            motor3.writeMicroseconds(1500);
            motor4.writeMicroseconds(1500);
        #elif defined USE_SERIAL
        	front_mc.write((uint8_t)0);
        	back_mc.write((uint8_t)0);
        #else
            REG_MOTOR_1 = 0;
            REG_MOTOR_2 = 0;
            REG_MOTOR_3 = 0;
            REG_MOTOR_4 = 0;
        #endif
    }
    else
    {
        #ifdef USE_SERVO
            motor1.writeMicroseconds(hs_control_msg.motor_power[0]);
            motor2.writeMicroseconds(hs_control_msg.motor_power[1]);
            motor3.writeMicroseconds(hs_control_msg.motor_power[2]);
            motor4.writeMicroseconds(hs_control_msg.motor_power[3]);
        #elif defined USE_SERIAL
        	front_mc.write(MAKE_MOTOR_1_BYTE(hs_control_msg.motor_power[0]));
        	front_mc.write(MAKE_MOTOR_2_BYTE(hs_control_msg.motor_power[1]));
        	back_mc.write(MAKE_MOTOR_3_BYTE(hs_control_msg.motor_power[2]));
        	back_mc.write(MAKE_MOTOR_4_BYTE(hs_control_msg.motor_power[3]));
        #else
            REG_MOTOR_1 = hs_control_msg.motor_power[0];
            REG_MOTOR_2 = hs_control_msg.motor_power[1];
            REG_MOTOR_3 = hs_control_msg.motor_power[2];
            REG_MOTOR_4 = hs_control_msg.motor_power[3];
        #endif
    }
    if (nh.connected())
    {
        //Read feedback values
        for (int l = 0; l < 4; l++)
        {
            long total = 0;
            current_samples[l][current_sample_ptr[l]] = ADC_TO_mA(analogRead(current_pins[l]));
            current_sample_ptr[l]++;
            if (current_sample_ptr[l] == CURRENT_AVERAGE_SAMPLES)
                current_sample_ptr[l] = 0;
            for (int m = 0; m < CURRENT_AVERAGE_SAMPLES; m++)
                total += current_samples[l][m];
            hs_feedback_msg.motor_current[l] = total/CURRENT_AVERAGE_SAMPLES;
            hs_feedback_msg.pod_position[l] = analogRead(encoder_pins[l]);
        }
        
        //Publish feedback message
        hs_feedback.publish(&hs_feedback_msg);
    }
}
//High speed control message handler
void recv_hs_control(const walrus_firmware_msgs::MainBoardHighSpeedControl& msg)
{
    hs_control_msg = msg;
    last_hs_msg = millis();
}

//Functions for low speed sensor readings staggered between high speed control operations
void doLowSpeedOperations()
{   
    uint8_t index;
    int16_t value;
    //Don't bother doing anything if we're not connected
    if (nh.connected())
    {
        //State machine for doing measurements between high speed control cycles
        switch(report_state)
        {
            case READ_TEMPHUMID:
                temphumid_sense.readAll();
                ls_data_msg.board_temp = temphumid_sense.getTemp();
                ls_data_msg.humidity = temphumid_sense.getHumidity();
                temphumid_sense.measure();
            break;
            case READ_PRESSURE:
                ls_data_msg.pressure = pressure_sense.readPressure();
            break;
            case READ_WATER:
                ls_data_msg.water_sense = ~(digitalRead(P_WATER_1) | 
                                        (digitalRead(P_WATER_2) << 1) | 
                                        (digitalRead(P_WATER_3) << 2) | 
                                        (digitalRead(P_WATER_4) << 3) | 
                                        (digitalRead(P_WATER_5) << 4) |
                                        (digitalRead(P_WATER_6) << 5));
            break;
            case READ_EXTTEMP_1:
            case READ_EXTTEMP_2:
            case READ_EXTTEMP_3:
            case READ_EXTTEMP_4: 
            case READ_EXTTEMP_5: 
            case READ_EXTTEMP_6: 
            case READ_EXTTEMP_7: 
            case READ_EXTTEMP_8: 
            case READ_EXTTEMP_9: 
            case READ_EXTTEMP_10:
                index = report_state - READ_EXTTEMP_1;
                exttemp_sense.reset();
                exttemp_sense.select(temp_addr[index]);
                exttemp_sense.write(READ_SCRATCH);
                for (int m = 0; m < 9; m++)
                    temp_buff[m] = exttemp_sense.read();
                ls_data_msg.temp_sense[index] = ((temp_buff[1] << 8) | temp_buff[0]) * 5;            
            break;
            case ISSUE_EXTTEMP_MEASURE:
                exttemp_sense.reset();
                exttemp_sense.skip();
                exttemp_sense.write(T_CONVERT);
            break;
            case READ_TENSION:
                extADC.sustain();
                ls_data_msg.tension[0] = extADC.getValue(CHAN_POT1);
                ls_data_msg.tension[1] = extADC.getValue(CHAN_POT2);
            break;
            case READ_BATT1_VOLTAGE:
            case READ_BATT2_VOLTAGE: 
            case READ_BATT3_VOLTAGE: 
            case READ_BATT4_VOLTAGE:
                index = report_state - READ_BATT1_VOLTAGE;
                upper[index].getVoltage(&value);
                ls_data_msg.ucell_voltage[index] = value;
                lower[index].getVoltage(&value);
                ls_data_msg.lcell_voltage[index] = value;
            break;
            case READ_BATT1_CURRENT:
            case READ_BATT2_CURRENT:
            case READ_BATT3_CURRENT: 
            case READ_BATT4_CURRENT:
                index = report_state - READ_BATT1_CURRENT;
                if (upper[index].getCurrent(&value))
                    ls_data_msg.batt_present |= (1 << index);
                else
                    ls_data_msg.batt_present &= ~(1 << index);
                ls_data_msg.ucell_current[index] = value;
                lower[index].getCurrent(&value);
                ls_data_msg.lcell_current[index] = value;
            break;
            case READ_BATT1_AVGCURRENT:
            case READ_BATT2_AVGCURRENT:
            case READ_BATT3_AVGCURRENT:
            case READ_BATT4_AVGCURRENT:
                index = report_state - READ_BATT1_AVGCURRENT;
                upper[index].getAvgCurrent(&value);
                ls_data_msg.ucell_avgcurr[index] = value; 
                lower[index].getAvgCurrent(&value);
                ls_data_msg.lcell_avgcurr[index] = value;
            break;    
            case READ_BATT1_TEMP:
            case READ_BATT2_TEMP:
            case READ_BATT3_TEMP:
            case READ_BATT4_TEMP:
                index = report_state - READ_BATT1_TEMP;
                upper[index].getTemp(&value);
                ls_data_msg.ucell_temp[index] = value;
                lower[index].getTemp(&value);
                ls_data_msg.lcell_temp[index] = value;
            break;
            case READ_BATT1_CHARGE:
            case READ_BATT2_CHARGE:
            case READ_BATT3_CHARGE:
            case READ_BATT4_CHARGE:
                index = report_state - READ_BATT1_CHARGE;
                upper[index].getCharge(&value);
                ls_data_msg.ucell_charge[index] = value;
                lower[index].getCharge(&value);
                ls_data_msg.lcell_charge[index] = value;
            break;
            case SEND_DATA:
                ls_data.publish(&ls_data_msg);
        }
        report_state++;
        if (report_state == LOW_SPEED_OP_STATE_NUM)
            report_state = 0;
    }
    else
        report_state = 0;
}

//Control message handler
void recv_control(const walrus_firmware_msgs::MainBoardControl& msg)
{
    switch (msg.type)
    {        
        case MainBoardControl::SET_ENABLE:
            output_disable = false;
        break;
        case MainBoardControl::SET_DISABLE:
            output_disable = true;
        break;
        case MainBoardControl::KEEP_ALIVE:
            //Don't need to do anything, other logic will take care of timeouts
        break;
        case MainBoardControl::SET_LED:
            #ifdef USE_SERVO
                analogWrite(led_pins[msg.index], msg.value / 4);
            #else
                switch (msg.index)
                {
                    case 0:
                        SET_LED1(msg.value);
                        break;
                    case 1:
                        SET_LED2(msg.value);
                        break;
                    case 2:
                        SET_LED3(msg.value);
                        break;
                }
            #endif
        break;
        case MainBoardControl::POWER_OFF:
            digitalWrite(P_CONTACTOR, HIGH);
        break;
        case MainBoardControl::POWER_ON:
            digitalWrite(P_CONTACTOR, LOW);
        break;
        case MainBoardControl::REQ_STATUS:
            to_pc_data.type = MainBoardControl::STATUS;
            to_pc_data.index = 0;
            to_pc_data.value = status->code;
            to_pc_data.msg = status->str;
            to_pc.publish(&to_pc_data);
        break;
        case MainBoardControl::REQ_BATT_INFO:
            if (output_disable)
            {
                char buff[30];
                delayed_operation = true;
                to_pc_data.index = msg.index;
                to_pc_data.value = 0;
                lower[msg.index].getManufacturer(buff, sizeof(buff));
                to_pc_data.type = MainBoardControl::BATT_MFR; 
                to_pc_data.msg = buff;  
                to_pc.publish(&to_pc_data);
                lower[msg.index].getDeviceName(buff, sizeof(buff));
                to_pc_data.type = MainBoardControl::BATT_NAME;
                to_pc_data.msg = buff;
                to_pc.publish(&to_pc_data);
                lower[msg.index].getChemistry(buff, sizeof(buff));
                to_pc_data.type = MainBoardControl::BATT_CHEM;
                to_pc_data.msg = buff;
                to_pc.publish(&to_pc_data);
            }
            else
            {
                mark_error = true;
                err_str = SLOW_REQUEST_ERROR;
            }
        break;
        default:
            mark_error = true;
            err_str = INVALID_REQUEST_ERROR;
        break;
    }
    no_ls = true;
    last_control_msg = millis();
}


void setup()
{
    //Issue a one-wire convert command to being conversion on all temperature sensors
    //Converting takes 750 ms so do this first so the first reading is valid
    exttemp_sense.reset();
    exttemp_sense.skip();
    exttemp_sense.write(T_CONVERT);
    
    //Initialize node, publishers and subscribers
    nh.initNode();
    nh.advertise(ls_data);
    nh.advertise(hs_feedback);
    nh.advertise(to_pc);
    nh.subscribe(hs_control);
    nh.subscribe(from_pc);
 
    //Setup digital IO  
    pinMode(P_WATER_1, INPUT_PULLUP);
    pinMode(P_WATER_2, INPUT_PULLUP);
    pinMode(P_WATER_3, INPUT_PULLUP);
    pinMode(P_WATER_4, INPUT_PULLUP);
    pinMode(P_WATER_5, INPUT_PULLUP);
    pinMode(P_WATER_6, INPUT_PULLUP);
    pinMode(P_EXT_LED_1, OUTPUT);
    pinMode(P_EXT_LED_2, OUTPUT);
    pinMode(P_EXT_LED_3, OUTPUT);
    pinMode(P_LED_STATUS, OUTPUT);
    pinMode(P_CONTACTOR, OUTPUT);
    
    //Setup external ADC
    extADC.begin(ADDR_EXT_ADC, 2);
    
    //Setup temperature/humidity sensor
    temphumid_sense.begin(ADDR_TEMP_HUMID);
    temphumid_sense.measure();
    
    //Setup Pressure sensor
    pressure_sense.begin();
    pressure_sense.setModeBarometer();
    pressure_sense.setOversampleRate(7); 
    pressure_sense.enableEventFlags();
    
    //Setup battery SMBus objects
    upper[0].begin(&i2c_bus0);
    upper[1].begin(&i2c_bus2);
    upper[2].begin(&i2c_bus4);
    upper[3].begin(&i2c_bus6);
    lower[0].begin(&i2c_bus1);
    lower[1].begin(&i2c_bus3);
    lower[2].begin(&i2c_bus5);
    lower[3].begin(&i2c_bus7);

    //Init current sensor averages
    for (int l = 0; l < 4; l++)
    {
        current_sample_ptr[l] = 0;
        for (int m = 0; m < CURRENT_AVERAGE_SAMPLES; m++)
            current_samples[l][m] = 0;
    }
    
    //Setup timer 2 to control LED 3
    PRR0 &= ~(1 << PRTIM2); //Clear power reduction disable
    OCR2A = (uint16_t)0; //Set output compare register to 0 (determines duty cycle)
    TCCR2A = (1 << COM2A1) | (1 << WGM20); //Set OCA pin to set pin on downcounting match and clear it on upcounting match
    TCCR2B = (1 << CS22) | (1 << CS21); //Set clock mode to clk/256 for a 62.5 khz counter
    //Setup timer 1 to control motor 3 and LED's 1 and 2 via output compare
    PRR0 &= ~(1 << PRTIM3); //Clear power reduction disable
    ICR3 = (uint16_t)TIMER_TOP; //Put 20,000 in input capture register (used as TOP for timer) to make 50 Hz period PWM
    OCR3B = (uint16_t)0;  //It turns out that the number in these registers equals the high pulse time of the signal in microseconds
    OCR3C = (uint16_t)0; 
    TCCR3A = (1 << COM3B1) | (1 << COM3C1); //Set all OC pins to set pin on downcounting match and clear it on upcounting match
    TCCR3B = (1 << WGM33) | (1 << CS31); //Set clock mode to clk/8 prescalar for a 2 Mhz counter
    
    
    //Setup Motors
    #ifdef USE_SERVO
        motor1.attach(P_MOTOR_1);
        motor2.attach(P_MOTOR_2);
        motor3.attach(P_MOTOR_3);
        motor4.attach(P_MOTOR_4);
        motor1.writeMicroseconds(1500);
        motor2.writeMicroseconds(1500);
        motor3.writeMicroseconds(1500);
        motor4.writeMicroseconds(1500);
    #elif defined USE_SERIAL
    	front_mc.begin(19200);
    	back_mc.begin(19200);
    #else
        //Setup timer 1 to control motors 1, 2 and 4 via output compare
        PRR0 &= ~(1 << PRTIM1); //Clear power reduction disable
        ICR1 = (uint16_t)TIMER_TOP; //Put 20,000 in input capture register (used as TOP for timer) to make 50 Hz period PWM
        OCR1A = (uint16_t)0;  //Set output compare registers to 0 (determines duty cycle)
        OCR1B = (uint16_t)0;  //It turns out that the number in these registers equals the high pulse time of the signal in microseconds
        OCR1C = (uint16_t)0; 
        TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << COM1C1); //Set all OC pins to set pin on downcounting match and clear it on upcounting match
        TCCR1B = (1 << WGM13) | (1 << CS11); //Set clock mode to clk/8 prescalar for a 2 Mhz counter
        TCCR3B |= (1 << COM3A1); //Set timer 3 to control motor 3
        OCR3A = (uint16_t)0;  //Set output compare registers to 0 (determines duty cycle)
        pinMode(P_MOTOR_1, OUTPUT);  //Set all motor pins to outputs
        pinMode(P_MOTOR_2, OUTPUT);
        pinMode(P_MOTOR_3, OUTPUT);
        pinMode(P_MOTOR_4, OUTPUT);       
    #endif
}

void loop()
{    
    long loop_start, elapsed;
    
    //Get start of loop time
    loop_start = millis();
    //Increment loop counter
    counter++;
    //Clear timing control
    no_ls = false;
    delayed_operation = false;

    //Check for current status
    //No PC connection at all
    if (!nh.connected())  
    {
        output_disable = true;
        status = &STATUS_NO_CONNECTION;
    }
    //PC is connected but not sending data (node probably not running)
    else if (millis() > last_control_msg + CONTROL_DATA_TIMEOUT)
    {
        output_disable = true;
        status = &STATUS_NO_CONTROL_DATA;
    }
    //Getting control data, but motors are disabled (don't expect high speed data)
    else if (output_disable)
        status = &STATUS_DISABLED;
    //Motors are enabled but no high speed data is being sent (should always get data when enabled)
    else if (millis() > last_hs_msg + HS_DATA_TIMEOUT)
    {
        output_disable = true;
        mark_error = true;
        err_str = NO_MOTOR_DATA_ERROR;     
    }    
    //Everything's cool and motors are enabled
    else
        status = &STATUS_ENABLED;
        
    //Blink status LED based on status
    if (LED_TRIGGER(counter))
    {
        int segment = LED_SEGMENT(counter);
        led_state = (status->blink_code & (1 << segment));
        digitalWrite(P_LED_STATUS, led_state);
    }
        
    //Listen for ros messages
    nh.spinOnce();    
        
    //Check if processing a low speed request has been disabled by the control message handler
    if (!no_ls)
    {
        //Issue a status change message
        if (status != last_status)
        {
            to_pc_data.type = MainBoardControl::STATUS;
            to_pc_data.index = 0;
            to_pc_data.value = status->code;
            to_pc_data.msg = status->str;
            to_pc.publish(&to_pc_data);
            last_status = status;
        }
        //Issue an error
        else if (mark_error)
        {
            to_pc_data.type = MainBoardControl::ERROR;
            to_pc_data.index = 0;
            to_pc_data.value = 0;
            to_pc_data.msg = err_str;
            to_pc.publish(&to_pc_data);
            mark_error = false;
        }
        //Do low speed sensor readings
        else
            doLowSpeedOperations();
    }
        
    //Do high speed motor control and feedback operations
    doHighSpeedOperations();    
    
    //Sleep until next cycle
    //This loop should never run longer than 20ms unless there is a delayed operation which is only allowed when disabled
    elapsed = millis() - loop_start;
    if (elapsed < ROS_MSG_RATE)    
        delay(ROS_MSG_RATE-elapsed);
    else if (!delayed_operation) 
    {
        mark_error = true;
        err_str = LOOP_TOO_LONG_ERROR;
    }
}

