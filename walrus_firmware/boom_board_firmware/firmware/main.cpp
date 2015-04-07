
#include <Arduino.h>
#include <ros.h>
#include <walrus_firmware_msgs/BoomBoardHighSpeedControl.h>
#include <walrus_firmware_msgs/BoomBoardHighSpeedFeedback.h>
#include <walrus_firmware_msgs/BoomBoardLowSpeedData.h>
#include <walrus_firmware_msgs/BoomBoardControl.h>
#include "constants.h"
#include <Wire.h>
#include <TempHumid.h>
#include <ExternalADC.h>
#include <Bridge.h>
#include <Maxon.h>

using namespace walrus_firmware_msgs;

//ROS node handle
ros::NodeHandle nh;

//Functions and messages for high speed motor control and feed back
bool output_disable = true;
long last_hs_msg = 0;
void enableOutput();
void disableOutput();
void doHighSpeedOperations();
void recv_hs_control(const walrus_firmware_msgs::BoomBoardHighSpeedControl& msg);
walrus_firmware_msgs::BoomBoardHighSpeedFeedback hs_feedback_msg;
walrus_firmware_msgs::BoomBoardHighSpeedControl hs_control_msg;
ros::Publisher hs_feedback("boom_board/hs_feedback", &hs_feedback_msg);
ros::Subscriber<walrus_firmware_msgs::BoomBoardHighSpeedControl> hs_control("boom_board/hs_control", &recv_hs_control);

//Do low speed motor control operations, stagger across cycles
void doLowSpeedOperations();
int report_state = 0;
walrus_firmware_msgs::BoomBoardLowSpeedData ls_data_msg;
ros::Publisher ls_data("boom_board/ls_data", &ls_data_msg);

//Control pipes
bool mark_error = false;
const char* err_str = "";
long last_control_msg = 0;
bool no_ls = false;
void recv_control(const walrus_firmware_msgs::BoomBoardControl& msg);
walrus_firmware_msgs::BoomBoardControl to_pc_data;
ros::Publisher to_pc("boom_board/board_to_PC_control", &to_pc_data);
ros::Subscriber<walrus_firmware_msgs::BoomBoardControl> from_pc("boom_board/PC_to_board_control", &recv_control);


//External ADC
ExternalADC extADC;
//Temperature and humididty sensor
TempHumid temphumid_sense;
//Maxon deploy motor controller
Maxon maxon;
//H-Bridge pan and tilt motor driver
Bridge bridge;
//Count the number of loops for timing
long counter = 0;
//Status of system (to show on LED)
Status* status = &STATUS_ENABLED;
Status* last_status = &STATUS_ENABLED;
bool led_state = false;

//Enable motor drivers
void enableOutput()
{
    maxon.enable();
    bridge.enable(CHAN_TILT_MOTOR);
    bridge.enable(CHAN_PAN_MOTOR);
    output_disable = false;
}
//Disable motor drivers
void disableOutput()
{
    //maxon.disable();
    //bridge.disable(CHAN_TILT_MOTOR);
    //bridge.disable(CHAN_PAN_MOTOR);
    output_disable = true;
}
//Functions for high speed motor control operations
void doHighSpeedOperations()
{
    //Sustain motor control leds
    bridge.sustain();
    maxon.sustain();
    if (nh.connected())
    {
        //Write motor powers
        bridge.setMotor(CHAN_TILT_MOTOR, hs_control_msg.tilt_power);
        bridge.setMotor(CHAN_PAN_MOTOR, hs_control_msg.pan_power);
        maxon.setMotor(hs_control_msg.deploy_power);
        
        //Read position potentiometers
        extADC.sustain();
        hs_feedback_msg.tilt_position = extADC.getValue(CHAN_TILT_POT);
        hs_feedback_msg.pan_position = extADC.getValue(CHAN_PAN_POT);
        hs_feedback_msg.deploy_position = extADC.getValue(CHAN_DEPLOY_POT);
        
        //Read current
        hs_feedback_msg.tilt_current = ADC_TO_mA(extADC.getValue(CHAN_TILT_CURRENT));
        hs_feedback_msg.pan_current = ADC_TO_mA(extADC.getValue(CHAN_PAN_CURRENT));
        
        //Publish feedback message
        hs_feedback.publish(&hs_feedback_msg);
    }
}
//High speed control message handler
void recv_hs_control(const walrus_firmware_msgs::BoomBoardHighSpeedControl& msg)
{
    bridge.setMotor(CHAN_TILT_MOTOR, msg.tilt_power);
        bridge.setMotor(CHAN_PAN_MOTOR, msg.pan_power);
        maxon.setMotor(msg.deploy_power);
    //hs_control_msg = msg;
    last_hs_msg = millis();
}

//Functions for low speed sensor readings staggered between high speed control operations
void doLowSpeedOperations()
{   
    if (nh.connected())
    {
        switch(report_state)
        {
            case READ_TEMPHUMID:
                temphumid_sense.readAll();
                ls_data_msg.temp = temphumid_sense.getTemp();
                ls_data_msg.humidity = temphumid_sense.getHumidity();
                temphumid_sense.measure();
            break;
            case READ_GAS:
                ls_data_msg.CO_sense = extADC.getValue(CHAN_CO_SENSE);
                ls_data_msg.LPG_sense = extADC.getValue(CHAN_LPG_SENSE);
                ls_data_msg.CNG_sense = extADC.getValue(CHAN_CNG_SENSE);
                ls_data_msg.H_sense = extADC.getValue(CHAN_H_SENSE);
            break;
            case READ_ANALOG:
                ls_data_msg.analog[0] = analogRead(P_ANALOG_EXP_1);
                ls_data_msg.analog[1] = analogRead(P_ANALOG_EXP_2);
                ls_data_msg.analog[2] = analogRead(P_ANALOG_EXP_3);
                ls_data_msg.analog[3] = analogRead(P_ANALOG_EXP_4);
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
void recv_control(const walrus_firmware_msgs::BoomBoardControl& msg)
{
    if (nh.connected())
    {
        switch (msg.type)
        {
            case BoomBoardControl::SET_ENABLE:
                enableOutput();
            break;
            case BoomBoardControl::SET_DISABLE:
                disableOutput();
            break;
            case BoomBoardControl::KEEP_ALIVE:
                //Don't need to do anything, other logic will take care of timeouts
            break;
            case BoomBoardControl::SET_CAM_LED:
                analogWrite(P_CAM_LED, msg.value);
            break;
            case BoomBoardControl::REQ_STATUS:
                to_pc_data.type = BoomBoardControl::STATUS;
                to_pc_data.value = status->code;
                to_pc_data.msg = status->str;
                to_pc.publish(&to_pc_data);
            break;
            default:
                mark_error = true;
                err_str = INVALID_REQUEST_ERROR;
            break;
        }
    }
    no_ls = true;
    last_control_msg = millis();
}


void setup()
{
    //Initialize node, publishers and subscribers
    nh.initNode();
    nh.advertise(ls_data);
    nh.advertise(hs_feedback);
    nh.advertise(to_pc);
    nh.subscribe(hs_control);
    nh.subscribe(from_pc);
    
    //Setup digital IO
    pinMode(P_LED_STATUS, OUTPUT);
    pinMode(P_CAM_LED, OUTPUT);
    pinMode(P_LPF_CLK, OUTPUT);
    
    //Output a clock to configure the ADC filters
    tone(P_LPF_CLK, LPF_VALUE);
    
    //Setup external ADC
    extADC.begin(ADDR_EXT_ADC, NUM_CHANNELS);
    
    //Setup temperature/humidity sensor
    temphumid_sense.begin(ADDR_TEMP_HUMID);
    temphumid_sense.measure();
    
    //Force setup timer 3 to work with analogWrite
    //This should not have to be done, but it dosn't work otherwise
    PRR0 &= ~(1 << PRTIM3); //Clear power reduction disable
    ICR3 = (uint16_t)255; //Put 20,000 in input capture register (used as TOP for timer) to make 50 Hz period PWM
    OCR3A = (uint16_t)0;  //Set output compare registers to 0 (determines duty cycle)
    OCR3B = (uint16_t)0;  //It turns out that the number in these registers equals the high pulse time of the signal in microseconds
    OCR3C = (uint16_t)0; 
    TCCR3A = 0;//(1 << COM3A1) | (1 << COM3B1) | (1 << COM3C1); //Set all OC pins to set pin on downcounting match and clear it on upcounting match
    TCCR3B = (1 << WGM33) | (1 << CS31); //Set clock mode to clk/8 prescalar for a 2 Mhz counter
    
    //Setup maxon motor controller
    maxon.begin(P_MAXON_IN1, P_MAXON_IN2, P_MAXON_DIR, P_MAXON_EN, P_MAXON_SPEED, P_MAXON_READY, P_MAXON_STATUS);
    maxon.setMode(SPEED_MODE_OPEN);
    maxon.setLEDDir(LED_SOURCE);
    maxon.disable();
    
    //Setup H-bridge motor controller
    bridge.begin(P_BRIDGE_IN1, P_BRIDGE_IN2, P_BRIDGE_IN3, P_BRIDGE_IN4, P_BRIDGE_D1, P_BRIDGE_D2, P_BRIDGE_D3, P_BRIDGE_D4, P_BRIDGE_SFA, P_BRIDGE_SFB, P_BRIDGE_LEDA, P_BRIDGE_LEDB);
    bridge.setLEDDir(LED_SOURCE);
    bridge.setBrake(CHAN_PAN_MOTOR);
    bridge.setBrake(CHAN_TILT_MOTOR);
    bridge.disable(CHAN_PAN_MOTOR);
    bridge.disable(CHAN_TILT_MOTOR);
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
    
    //Check for current status
    //No PC connection at all
    if (!nh.connected())  
    {
        disableOutput();
        status = &STATUS_NO_CONNECTION;
    }
    //PC is connected but not sending data (node probably not running)
    else if (millis() > last_control_msg + CONTROL_DATA_TIMEOUT)
    {
        disableOutput();
        status = &STATUS_NO_CONTROL_DATA;
    }
    //Getting control data, but motors are disabled (don't expect high speed data)
    else if (output_disable)
        status = &STATUS_DISABLED;
    //Motors are enabled but no high speed data is being sent (should always get data when enabled)
    else if (millis() > last_hs_msg + HS_DATA_TIMEOUT)
    {
        disableOutput();
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
            to_pc_data.type = BoomBoardControl::STATUS;
            to_pc_data.value = status->code;
            to_pc_data.msg = status->str;
            to_pc.publish(&to_pc_data);
            last_status = status;
        }
        //Issue an error
        else if (mark_error)
        {
            to_pc_data.type = BoomBoardControl::ERROR;
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
    //This loop should never run longer than 20ms 
    elapsed = millis() - loop_start;
    if (elapsed < ROS_MSG_RATE)    
        delay(ROS_MSG_RATE-elapsed);
    else
    {
        mark_error = true;
        err_str = LOOP_TOO_LONG_ERROR;
    }
}

