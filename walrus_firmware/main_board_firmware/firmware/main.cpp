
#include <Arduino.h>
#include <ros.h>
#include <walrus_firmware_msgs/MainBoardPodMotorFeedback.h>
#include <walrus_firmware_msgs/MainBoardSensorData.h>
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
void doHighSpeedOperations();
walrus_firmware_msgs::MainBoardPodMotorFeedback pm_feedback_msg;
ros::Publisher pm_feedback("main_board/pm_feedback", &pm_feedback_msg);

//Do low speed motor control operations, stagger across cycles
void doLowSpeedOperations();
int report_state = 0;
walrus_firmware_msgs::MainBoardSensorData sensor_data_msg;
ros::Publisher sensor_data("main_board/sensor_data", &sensor_data_msg);

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
//Battery SMBus objects
SmartBatt upper[4], lower[4];
//LED pin array
uint8_t encoder_pins[] = {P_ENCODER_1, P_ENCODER_2, P_ENCODER_3, P_ENCODER_4};
uint8_t led_pins[] = {P_EXT_LED_1, P_EXT_LED_2, P_EXT_LED_3};
//Temp sensor addresses
uint8_t temp_addr[10][8] = {TEMP_1_ADDR, TEMP_2_ADDR, TEMP_3_ADDR, TEMP_4_ADDR, TEMP_5_ADDR, TEMP_6_ADDR, TEMP_7_ADDR, TEMP_8_ADDR, TEMP_9_ADDR, TEMP_10_ADDR};
//Temp sensor buffer;
uint8_t temp_buff[9];
//Count the number of loops for timing
long counter = 0;
//Status of system (to show on LED)
uint16_t status = STATUS_NO_CONNECTION;
bool led_state = false;


//Functions for high speed motor control operations
void doHighSpeedOperations()
{
    if (nh.connected())
    {
        //Read feedback values
        for (int l = 0; l < 4; l++)
            pm_feedback_msg.pod_position[l] = analogRead(encoder_pins[l]);
        
        //Publish feedback message
        pm_feedback.publish(&pm_feedback_msg);
    }
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
                sensor_data_msg.board_temp = temphumid_sense.getTemp();
                sensor_data_msg.humidity = temphumid_sense.getHumidity();
                temphumid_sense.measure();
            break;
            case READ_PRESSURE:
                sensor_data_msg.pressure = pressure_sense.readPressure();
            break;
            case READ_WATER:
                sensor_data_msg.water_sense = ~(digitalRead(P_WATER_1) | 
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
                sensor_data_msg.temp_sense[index] = ((temp_buff[1] << 8) | temp_buff[0]) * 5;            
            break;
            case ISSUE_EXTTEMP_MEASURE:
                exttemp_sense.reset();
                exttemp_sense.skip();
                exttemp_sense.write(T_CONVERT);
            break;
            case READ_TENSION:
                extADC.sustain();
                sensor_data_msg.tension[0] = extADC.getValue(CHAN_POT1);
                sensor_data_msg.tension[1] = extADC.getValue(CHAN_POT2);
            break;
            case READ_BATT1_VOLTAGE:
            case READ_BATT2_VOLTAGE: 
            case READ_BATT3_VOLTAGE: 
            case READ_BATT4_VOLTAGE:
                index = report_state - READ_BATT1_VOLTAGE;
                upper[index].getVoltage(&value);
                sensor_data_msg.ucell_voltage[index] = value;
                lower[index].getVoltage(&value);
                sensor_data_msg.lcell_voltage[index] = value;
            break;
            case READ_BATT1_CURRENT:
            case READ_BATT2_CURRENT:
            case READ_BATT3_CURRENT: 
            case READ_BATT4_CURRENT:
                index = report_state - READ_BATT1_CURRENT;
                if (upper[index].getCurrent(&value))
                    sensor_data_msg.batt_present |= (1 << index);
                else
                    sensor_data_msg.batt_present &= ~(1 << index);
                sensor_data_msg.ucell_current[index] = value;
                lower[index].getCurrent(&value);
                sensor_data_msg.lcell_current[index] = value;
            break;
            case READ_BATT1_AVGCURRENT:
            case READ_BATT2_AVGCURRENT:
            case READ_BATT3_AVGCURRENT:
            case READ_BATT4_AVGCURRENT:
                index = report_state - READ_BATT1_AVGCURRENT;
                upper[index].getAvgCurrent(&value);
                sensor_data_msg.ucell_avgcurr[index] = value; 
                lower[index].getAvgCurrent(&value);
                sensor_data_msg.lcell_avgcurr[index] = value;
            break;    
            case READ_BATT1_TEMP:
            case READ_BATT2_TEMP:
            case READ_BATT3_TEMP:
            case READ_BATT4_TEMP:
                index = report_state - READ_BATT1_TEMP;
                upper[index].getTemp(&value);
                sensor_data_msg.ucell_temp[index] = value;
                lower[index].getTemp(&value);
                sensor_data_msg.lcell_temp[index] = value;
            break;
            case READ_BATT1_CHARGE:
            case READ_BATT2_CHARGE:
            case READ_BATT3_CHARGE:
            case READ_BATT4_CHARGE:
                index = report_state - READ_BATT1_CHARGE;
                upper[index].getCharge(&value);
                sensor_data_msg.ucell_charge[index] = value;
                lower[index].getCharge(&value);
                sensor_data_msg.lcell_charge[index] = value;
            break;
            case SEND_DATA:
                sensor_data.publish(&sensor_data_msg);
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
        case MainBoardControl::KEEP_ALIVE:
            //Don't need to do anything, other logic will take care of timeouts
        break;
        case MainBoardControl::SET_LED:
            analogWrite(led_pins[msg.index], msg.value);
        break;
        case MainBoardControl::POWER_OFF:
            digitalWrite(P_CONTACTOR, HIGH);
        break;
        case MainBoardControl::POWER_ON:
            digitalWrite(P_CONTACTOR, LOW);
        break;
        case MainBoardControl::REQ_BATT_INFO:
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
    nh.advertise(sensor_data);
    nh.advertise(pm_feedback);
    nh.advertise(to_pc);
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
        status = STATUS_NO_CONNECTION;
    //PC is connected but not sending data (node probably not running)
    else if (millis() > last_control_msg + CONTROL_DATA_TIMEOUT)
        status = STATUS_NO_CONTROL_DATA;
    //Getting control data, but motors are disabled (don't expect high speed data)
    else
        status = STATUS_OK;
        
    //Blink status LED based on status
    if (LED_TRIGGER(counter))
    {
        int segment = LED_SEGMENT(counter);
        led_state = (status & (1 << segment));
        digitalWrite(P_LED_STATUS, led_state);
    }
        
    //Listen for ros messages
    nh.spinOnce();    
        
    //Check if processing a low speed request has been disabled by the control message handler
    if (!no_ls)
    {
        //Issue an error
        if (mark_error)
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

