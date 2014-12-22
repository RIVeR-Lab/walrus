#include <Servo.h>

/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Empty.h>
#include <Arduino.h>

Servo left_drive1;
Servo right_drive1;
Servo left_drive2;
Servo right_drive2;
Servo fl_pod;
Servo fr_pod;
Servo bl_pod;
Servo br_pod;

void left_driveCB( const std_msgs::UInt16& pulse)
{
  left_drive1.writeMicroseconds(pulse.data);
  left_drive2.writeMicroseconds(pulse.data);
}

void right_driveCB( const std_msgs::UInt16& pulse)
{
  right_drive1.writeMicroseconds(pulse.data);
  right_drive2.writeMicroseconds(pulse.data);
}

void fl_podCB( const std_msgs::UInt16& pulse)
{
   fl_pod.writeMicroseconds(pulse.data);
}

void fr_podCB( const std_msgs::UInt16& pulse)
{
   fr_pod.writeMicroseconds(pulse.data);
}

void bl_podCB( const std_msgs::UInt16& pulse)
{
   bl_pod.writeMicroseconds(pulse.data);
}

void br_podCB( const std_msgs::UInt16& pulse)
{
   br_pod.writeMicroseconds(pulse.data);
}

ros::NodeHandle  nh;
ros::Subscriber<std_msgs::UInt16> left_drive_sub("/calf/left_drive", left_driveCB);
ros::Subscriber<std_msgs::UInt16> right_drive_sub("/calf/right_drive", right_driveCB);
ros::Subscriber<std_msgs::UInt16> fr_pod_sub("/calf/front_right_pod", fr_podCB);
ros::Subscriber<std_msgs::UInt16> br_pod_sub("/calf/back_right_pod", br_podCB);
ros::Subscriber<std_msgs::UInt16> bl_pod_sub("/calf/back_left_pod", bl_podCB);
ros::Subscriber<std_msgs::UInt16> fl_pod_sub("/calf/front_left_pod", fl_podCB);

std_msgs::Empty beat;
ros::Publisher heartbeat("/calf/heartbeat", &beat);

void setup()
{
  left_drive1.attach(10);
  left_drive2.attach(11);
  right_drive1.attach(12);
  right_drive2.attach(13);
  fr_pod.attach(9);
  br_pod.attach(8);
  bl_pod.attach(7);
  fl_pod.attach(6);
  nh.initNode();
  nh.advertise(heartbeat);
  nh.subscribe(left_drive_sub);
  nh.subscribe(right_drive_sub);
  nh.subscribe(fl_pod_sub);
  nh.subscribe(fr_pod_sub);
  nh.subscribe(bl_pod_sub);
  nh.subscribe(br_pod_sub);
}

void loop()
{
  heartbeat.publish(&beat);
  nh.spinOnce();
  delay(90);
}
