#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16
from sensor_msgs.msg import Joy
import math
from operator import add

def joystick_callback(data):
        print data
        global last_update
        global front_left_pub
        global front_right_pub
        global back_left_pub
        global back_right_pub
        global left_drive_pub
        global right_drive_pub

        time = rospy.get_rostime()
        if last_update.secs==0 and last_update.nsecs==0:
                dt = 0
        else:
                dt = (time-last_update).to_sec()
        last_update = time

        left = (-data.axes[1]*500) + 1500;
        right = (-data.axes[3]*500) + 1500;
	back = data.buttons[1]
	if (back):
		fl_pod = 1500
		fr_pod = 1500
		bl_pod = 1500 + (500*data.buttons[4]) + (500*-data.buttons[6])
		br_pod = 1500 + (500*data.buttons[5]) + (500*-data.buttons[7])
	else:
		fl_pod = 1500 + (500*data.buttons[4]) + (500*-data.buttons[6])
		fr_pod = 1500 + (500*data.buttons[5]) + (500*-data.buttons[7])
		bl_pod = 1500
		br_pod = 1500

        left_drive_pub.publish(left)
	right_drive_pub.publish(right) 
        front_left_pub.publish(fl_pod)
        front_right_pub.publish(fr_pod)
        back_left_pub.publish(bl_pod)
        back_right_pub.publish(br_pod)

    
def main():
        rospy.init_node('thigh_teleop', anonymous=True)

        global last_update
        last_update = rospy.get_rostime()

        global front_left_pub
        global front_right_pub
        global back_left_pub
        global back_right_pub
        global left_drive_pub
        global right_drive_pub
        front_left_pub = rospy.Publisher('front_left_pod', UInt16, 10)
        front_right_pub = rospy.Publisher('front_right_pod', UInt16, 10)
        back_left_pub = rospy.Publisher('back_left_pod', UInt16, 10)
        back_right_pub = rospy.Publisher('back_right_pod', UInt16, 10)
        
        left_drive_pub = rospy.Publisher('left_drive', UInt16, 10)
        right_drive_pub = rospy.Publisher('right_drive', UInt16, 10)
        
        rospy.Subscriber("joy", Joy, joystick_callback)
        
        rospy.spin()
        
if __name__ == '__main__':
        main()
