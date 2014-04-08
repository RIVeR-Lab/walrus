#!/usr/bin/env python
# license removed for brevity
from __future__ import division
import rospy
from std_msgs.msg import Float64
from math import sin

def talker():
    front_left_pub = rospy.Publisher('/walrus/front_left_pod_joint_controller/command', Float64)
    front_right_pub = rospy.Publisher('/walrus/front_right_pod_joint_controller/command', Float64)
    back_left_pub = rospy.Publisher('/walrus/back_left_pod_joint_controller/command', Float64)
    back_right_pub = rospy.Publisher('/walrus/back_right_pod_joint_controller/command', Float64)

    left_drive_pub = rospy.Publisher('/walrus/left_drive_controller/command', Float64)
    right_drive_pub = rospy.Publisher('/walrus/right_drive_controller/command', Float64)

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
        left_drive_pub.publish(sin(i/40))
        right_drive_pub.publish(sin(i/40))
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
