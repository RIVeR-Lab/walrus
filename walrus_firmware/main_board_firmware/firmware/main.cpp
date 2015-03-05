
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
#include <Servo.h>
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
ros::Publisher hs_feedback("/walrus/main_board/hs_feedback", &hs_feedback_msg);
ros::Subscriber<walrus_firmware_msgs::MainBoardHighSpeedControl> hs_control("/walrus/main_board/hs_control", &recv_hs_control);

//Do low speed motor control operations, stagger across cycles
void doLowSpeedOperations();
int report_state = 0;
walrus_firmware_msgs::MainBoardLowSpeedData ls_data_msg;
ros::Publisher ls_data("/walrus/main_board/ls_data", &ls_data_msg);

//Control pipes
bool mark_error = false;
char* err_str = "";
long last_control_msg = 0;
bool delayed_operation = false;
bool no_ls = false;
void recv_control(const walrus_firmware_msgs::MainBoardControl& msg);
walrus_firmware_msgs::MainBoardControl to_pc_data;
ros::Publisher to_pc("/walrus/main_board/board_to_PC_control", &to_pc_data);
ros::Subscriber<walrus_firmware_msgs::MainBoardControl> from_pc("/walrus/main_board/PC_to_board_control", &recv_control);


//Pressure sensor
MPL3115A2 pressure_sense;
//External ADC
ExternalADC extADC;
//Temperature and humidity sensor
TempHumid temphumid_sense;
//OneWire object for temperature sensors
OneWire exttemp_sense(P_EXT_TEMP);
//Motor servos
Servo motor1, motor2, motor3, motor4;
//Battery SMBus objects
SmartBatt upper[4], lower[4];
//LED pin array
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
        motor1.writeMicroseconds(0);
        motor2.writeMicroseconds(0);
        motor3.writeMicroseconds(0);
        motor4.writeMicroseconds(0);
    }
    else
    {
        motor1.writeMicroseconds(hs_control_msg.motor_power[0]);
        motor2.writeMicroseconds(hs_control_msg.motor_power[1]);
        motor3.writeMicroseconds(hs_control_msg.motor_power[2]);
        motor4.writeMicroseconds(hs_control_msg.motor_power[3]);
    }
    if (nh.connected())
    {
        //Read feedback values
        hs_feedback_msg.motor_current[0] = ADC_TO_mA(analogRead(P_CURRENT_1));
        hs_feedback_msg.motor_current[1] = ADC_TO_mA(analogRead(P_CURRENT_2));
        hs_feedback_msg.motor_current[2] = ADC_TO_mA(analogRead(P_CURRENT_3));
        hs_feedback_msg.motor_current[3] = ADC_TO_mA(analogRead(P_CURRENT_4));
        hs_feedback_msg.pod_position[0] = analogRead(P_ENCODER_1);
        hs_feedback_msg.pod_position[1] = analogRead(P_ENCODER_2);
        hs_feedback_msg.pod_position[2] = analogRead(P_ENCODER_3);
        hs_feedback_msg.pod_position[3] = analogRead(P_ENCODER_4);
        
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
                ls_data_msg.water_sense = digitalRead(P_WATER_1) | 
                                        (digitalRead(P_WATER_2) << 1) | 
                                        (digitalRead(P_WATER_3) << 2) | 
                                        (digitalRead(P_WATER_4) << 3) | 
                                        (digitalRead(P_WATER_5) << 4) |
                                        (digitalRead(P_WATER_6) << 5);
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
                ls_data_msg.ucell_voltage[index] = upper[index].getVoltage();
                ls_data_msg.lcell_voltage[index] = lower[index].getVoltage();
            break;
            case READ_BATT1_CURRENT:
            case READ_BATT2_CURRENT:
            case READ_BATT3_CURRENT: 
            case READ_BATT4_CURRENT:
                index = report_state - READ_BATT1_CURRENT;
                ls_data_msg.ucell_current[index] = upper[index].getCurrent();
                ls_data_msg.lcell_current[index] = lower[index].getCurrent();
            break;
            case READ_BATT1_AVGCURRENT:
            case READ_BATT2_AVGCURRENT:
            case READ_BATT3_AVGCURRENT:
            case READ_BATT4_AVGCURRENT:
                index = report_state - READ_BATT1_AVGCURRENT;
                ls_data_msg.ucell_avgcurr[index] = upper[index].getAvgCurrent();
                ls_data_msg.lcell_avgcurr[index] = lower[index].getAvgCurrent();
            break;    
            case READ_BATT1_TEMP:
            case READ_BATT2_TEMP:
            case READ_BATT3_TEMP:
            case READ_BATT4_TEMP:
                index = report_state - READ_BATT1_TEMP;
                ls_data_msg.ucell_temp[index] = upper[index].getTemp();
                ls_data_msg.lcell_temp[index] = lower[index].getTemp();
            break;
            case READ_BATT1_CHARGE:
            case READ_BATT2_CHARGE:
            case READ_BATT3_CHARGE:
            case READ_BATT4_CHARGE:
                index = report_state - READ_BATT1_CHARGE;
                ls_data_msg.ucell_charge[index] = upper[index].getCharge();
                ls_data_msg.lcell_charge[index] = lower[index].getCharge();
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
            analogWrite(led_pins[msg.index], msg.value & 0xFF);
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
                to_pc_data.msg = buff;
                lower[msg.index].getManufacturer(buff, 30);
                to_pc_data.type = MainBoardControl::BATT_MFR;   
                to_pc.publish(&to_pc_data);
                lower[msg.index].getDeviceName(buff, 30);
                to_pc_data.type = MainBoardControl::BATT_NAME;
                to_pc.publish(&to_pc_data);
                lower[msg.index].getChemistry(buff, 30);
                to_pc_data.type = MainBoardControl::BATT_CHEM;
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
    
    //Setup Motors
    motor1.attach(P_MOTOR_1);
    motor2.attach(P_MOTOR_2);
    motor3.attach(P_MOTOR_3);
    motor4.attach(P_MOTOR_4);
    motor1.writeMicroseconds(0);
    motor2.writeMicroseconds(0);
    motor3.writeMicroseconds(0);
    motor4.writeMicroseconds(0);

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
    
    //Setup timer 1 to control motors 1, 2 and 4 via output compare
    /*PRR0 &= ~(1 << PRTIM1); //Clear power reduction disable
    ICR1 = (uint16_t)20000; //Put 20,000 in input capture register (used as TOP for timer) to make 50 Hz period PWM
    OCR1A = (uint16_t)0;  //Set output compare registers to 0 (determines duty cycle)
    OCR1B = (uint16_t)0;  //It turns out that the number in these registers equals the high pulse time of the signal in microseconds
    OCR1C = (uint16_t)0; 
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << COM1C1); //Set all OC pins to set pin on downcounting match and clear it on upcounting match
    TCCR1B = (1 << WGM13) | (1 << CS11); //Set clock mode to clk/8 prescalar for a 2 Mhz counter
    pinMode(P_MOTOR_1, OUTPUT);  //Set all motor pins to outputs
    pinMode(P_MOTOR_2, OUTPUT);
    pinMode(P_MOTOR_3, OUTPUT);
    pinMode(P_MOTOR_4, OUTPUT);
    REG_MOTOR_1 = 1500;
    REG_MOTOR_2 = 1500;
    REG_MOTOR_4 = 1500;*/
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
        err_str = NO_MOTOR_DATA_ERROR;
        status = &STATUS_NO_HS_DATA;
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

