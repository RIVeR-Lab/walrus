#!/usr/bin/env python
# license removed for brevity
from __future__ import division
import rospy
from std_msgs.msg import Float64
from math import sin

def talker():
    front_left_pub = rospy.Publisher('/calf/front_left_pod_joint_controller/command', Float64)
    front_right_pub = rospy.Publisher('/calf/front_right_pod_joint_controller/command', Float64)
    back_left_pub = rospy.Publisher('/calf/back_left_pod_joint_controller/command', Float64)
    back_right_pub = rospy.Publisher('/calf/back_right_pod_joint_controller/command', Float64)

    left_drive_pub = rospy.Publisher('/calf/left_drive_controller/command', Float64)
    right_drive_pub = rospy.Publisher('/calf/right_drive_controller/command', Float64)

    rospy.init_node('test_bounce', anonymous=True)

    i = 0;
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        i += 1
        val = sin(i/20)/2+0.75;
        front_left_pub.publish(val)
        front_right_pub.publish(val)
        back_left_pub.publish(-val)
        back_right_pub.publish(-val)
        left_drive_pub.publish(sin(i/40)*3)
        right_drive_pub.publish(sin(i/40)*3)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
