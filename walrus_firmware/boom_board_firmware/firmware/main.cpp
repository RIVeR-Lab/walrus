
#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <walrus_firmware_msgs/BoomBoardTXMsg.h>
#include <walrus_firmware_msgs/BoomBoardRXMsg.h>
#include "constants.h"
#include <Wire.h>
#include <TempHumid.h>
#include <ExternalADC.h>
#include <Bridge.h>
#include <Maxon.h>

//void recv_msg(const walrus_firmware_msgs::BoomBoardTXMsg& msg);
//void disable();

//ROS node handle
//ros::NodeHandle nh;
//ROS Message publisher
//walrus_firmware_msgs::BoomBoardRXMsg rx_msg;
//ros::Publisher rx("walrus/main_board/rx", &rx_msg);
//ROS Message subscriber
//ros::Subscriber<walrus_firmware_msgs::BoomBoardTXMsg> tx("/walrus/boom_board/tx", &recv_msg);

//External ADC
ExternalADC extADC;
//Temperature and humididty sensor
TempHumid temphumid_sense;
//Maxon deploy motor controller
Maxon maxon;
//H-Bridge pan and tilt motor driver
Bridge bridge;
//Time in millisencods of the last received message
long last_msg = 0; 
//Count the number of loops for timing
long counter = 1;
//Status of system (to show on LED)
int status = STATUS_NO_PC;
bool led_state = HIGH;


//Receive TX message from ROS master
/*void recv_msg(const walrus_firmware_msgs::BoomBoardTXMsg &msg)
{
    //Set motor speeds
    bridge.setMotor(CHAN_PAN_MOTOR, msg.pan_power);
    bridge.setMotor(CHAN_TILT_MOTOR, msg.tilt_power);
    maxon.setMotor(msg.deploy_power);
    //Set cam LED intensity
    analogWrite(P_CAM_LED, msg.LED_intensity);
    //Mark receive time
    last_msg = millis();
}

//Stop all motors and turn off cam LED
void disable()
{
    //Disable all motors
    bridge.setMotor(CHAN_PAN_MOTOR, 0);
    bridge.setMotor(CHAN_TILT_MOTOR, 0);
    bridge.disable(CHAN_PAN_MOTOR);
    bridge.disable(CHAN_TILT_MOTOR);
    maxon.setMotor(0);
    maxon.disable();
    //Turn CAM LED off
    analogWrite(P_CAM_LED, 0);
}*/

void setup()
{
    Serial.begin(115200);
    //Initialize node, publishers and subscribers
    //nh.initNode();
    //nh.advertise(rx);
    //nh.subscribe(tx);
    
    //Setup digital IO
    pinMode(P_LED_STATUS, OUTPUT);
    pinMode(P_CAM_LED, OUTPUT);
    pinMode(P_LPF_CLK, OUTPUT);
    tone(P_LPF_CLK, LPF_VALUE);
    
    pinMode(0, OUTPUT);
    pinMode(1, OUTPUT);
    digitalWrite(1, LOW);
    delayMicroseconds(10);
    digitalWrite(0, LOW);
    delayMicroseconds(10);
    digitalWrite(0, HIGH);
    delayMicroseconds(10);
    digitalWrite(1, HIGH);
    delayMicroseconds(10); 
    pinMode(0, INPUT);
    pinMode(1, INPUT);
    
    //Setup external ADC
    //extADC.begin(ADDR_EXT_ADC, NUM_CHANNELS);
    
    //Setup temperature/humidity sensor
    temphumid_sense.begin(ADDR_TEMP_HUMID);
    
    //Setup maxon motor controller
    maxon.begin(P_MAXON_IN1, P_MAXON_IN2, P_MAXON_DIR, P_MAXON_EN, P_MAXON_SPEED, P_MAXON_READY, P_MAXON_STATUS);
    maxon.setMode(SPEED_MODE_OPEN);
    maxon.setLEDDir(LED_SOURCE);
    maxon.enable();
    
    //Setup H-bridge motor controller
    bridge.begin(P_BRIDGE_IN1, P_BRIDGE_IN2, P_BRIDGE_IN3, P_BRIDGE_IN4, P_BRIDGE_D1, P_BRIDGE_D2, P_BRIDGE_D3, P_BRIDGE_D4, P_BRIDGE_SFA, P_BRIDGE_SFB, P_BRIDGE_LEDA, P_BRIDGE_LEDB);
    bridge.setLEDDir(LED_SOURCE);
    bridge.setBrake(CHAN_PAN_MOTOR);
    bridge.setBrake(CHAN_TILT_MOTOR);
    bridge.enable(CHAN_PAN_MOTOR);
    bridge.enable(CHAN_TILT_MOTOR);
    
    //Setup LPF Filter
    //analogWrite(P_LPF_CLK, LPF_VALUE);
}

void loop()
{
    long loop_start, elapsed;
    //Mark time of loop start
    loop_start = millis();
    
    //Disable motors if we l PC connection
    /*if (!nh.connected())
    {
        //disable();
        status = STATUS_NO_PC;
    }
    //Disable motors if we don't receive a message in a specific amount of time
    else if (millis() > last_msg + MOTOR_OFF_TIMEOUT)
    {
        //disable();
        status = STATUS_NO_MSG;
    }
    else
    {
        //bridge.enable(CHAN_PAN_MOTOR);
        //bridge.enable(CHAN_TILT_MOTOR);
        //maxon.enable();
        status = STATUS_OK;    
    }*/
    
    //Blink status LED based on status
    if (counter % (status/ROS_MSG_RATE) == 0)
    {
        led_state = !led_state;
        /*Serial.print("CO: ");
        Serial.print(extADC.getValue(CHAN_CO_SENSE));
        Serial.print(" LPG: ");
        Serial.print(extADC.getValue(CHAN_LPG_SENSE));
        Serial.print(" H: ");
        Serial.print(extADC.getValue(CHAN_H_SENSE));
        Serial.print(" CNG: ");
        Serial.print(extADC.getValue(CHAN_CNG_SENSE));*/
        Serial.print(temphumid_sense.getTemp());
        Serial.print(" ");
        Serial.print(temphumid_sense.getHumidity());
        Serial.print("\r\n");
        digitalWrite(P_LED_STATUS, led_state);
    }
    
    //maxon.setMotor(extADC.getValue(CHAN_DEPLOY_POT)/16 - 128);
    //bridge.setMotor(CHAN_TILT_MOTOR, extADC.getValue(CHAN_TILT_POT)/16 - 128);
    //bridge.setMotor(CHAN_PAN_MOTOR, extADC.getValue(CHAN_PAN_POT)/16 - 128);
    //analogWrite(P_CAM_LED, extADC.getValue(CHAN_DEPLOY_POT)/16);
    
    //Read in external ADC samples
    //extADC.sustain();
    //Sustain maxon led status
    maxon.sustain();
    //Sustain H-bridge led status
    bridge.sustain();
    
    //Read in analog expansion samples
    //rx_msg.analog[0] = analogRead(P_ANALOG_EXP_1);
    /*rx_msg.analog[1] = analogRead(P_ANALOG_EXP_2);
    rx_msg.analog[2] = analogRead(P_ANALOG_EXP_3);
    rx_msg.analog[3] = analogRead(P_ANALOG_EXP_4);
    //Read in potentiometers
    rx_msg.deploy_position = extADC.getValue(CHAN_DEPLOY_POT);
    rx_msg.tilt_position = extADC.getValue(CHAN_TILT_POT);
    rx_msg.pan_position = extADC.getValue(CHAN_DEPLOY_POT);
    //Read in gas sensors
    rx_msg.CO_sense = extADC.getValue(CHAN_CO_SENSE);
    rx_msg.LPG_sense = extADC.getValue(CHAN_LPG_SENSE);
    rx_msg.H_sense = extADC.getValue(CHAN_H_SENSE);
    rx_msg.CNG_sense = extADC.getValue(CHAN_CNG_SENSE);
    //Read in temperature
    rx_msg.temp = temphumid_sense.getTemp();
    rx_msg.humidity = temphumid_sense.getHumidity();
    //Read in motor currents
    rx_msg.pan_current = extADC.getValue(CHAN_PAN_CURRENT);
    rx_msg.tilt_current = extADC.getValue(CHAN_TILT_CURRENT);*/
    //Publish our message
    //rx.publish(&rx_msg);
    
    //Allow ros to receive
    //nh.spinOnce();
    //Sleep until next cycle;
    elapsed = millis()-loop_start;
    if (elapsed < ROS_MSG_RATE)
        delay(ROS_MSG_RATE-elapsed);
    //Increment loop counter
    counter++;    
}


