#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
import math
from operator import add

def joystick_callback(data):
        print data
        global last_update
        global joint_state
        global drive_state
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

        left = data.axes[1];
        right = data.axes[3];
        
        reset_joints = data.buttons[0];
        normalize_joints = data.buttons[1];
        drive_joints = data.buttons[2];

        mod_joints = [data.buttons[4], data.buttons[5], data.buttons[6], data.buttons[7]];

        if any(mod_joints):
                joint_delta = left/2; #rad/s
                joint_state = map(add, joint_state, [mod*joint_delta*dt for mod in mod_joints])
                left_drive_pub.publish(0)
                right_drive_pub.publish(0)
        else:
                left_drive_pub.publish(-20*left)
                right_drive_pub.publish(-20*right)

        if not any(mod_joints):
                mod_joints = [1, 1, 1, 1]
                
        if reset_joints:
                joint_state = map(lambda state, mod: 0 if mod else state, joint_state, mod_joints)
                
        if drive_joints:
                joint_state = [-math.pi/4, -math.pi/4, -math.pi+0.05, -math.pi+0.05]
                
        if normalize_joints:
                values = map(lambda state, mod: mod*state, joint_state, mod_joints)
                avg = sum(values)/mod_joints.count(1);
                joint_state = map(lambda state, mod: avg if mod else state, joint_state, mod_joints)

        front_left_pub.publish(joint_state[0])
        front_right_pub.publish(joint_state[1])
        back_left_pub.publish(-joint_state[2])
        back_right_pub.publish(-joint_state[3])

    
def main():
        rospy.init_node('simple_teleop', anonymous=True)

        global joint_state
        joint_state = [0, 0, 0, 0]
        global last_update
        last_update = rospy.get_rostime()

        global front_left_pub
        global front_right_pub
        global back_left_pub
        global back_right_pub
        global left_drive_pub
        global right_drive_pub
        front_left_pub = rospy.Publisher('front_left_pod_joint_controller/command', Float64)
        front_right_pub = rospy.Publisher('front_right_pod_joint_controller/command', Float64)
        back_left_pub = rospy.Publisher('back_left_pod_joint_controller/command', Float64)
        back_right_pub = rospy.Publisher('back_right_pod_joint_controller/command', Float64)
        
        left_drive_pub = rospy.Publisher('left_drive_controller/command', Float64)
        right_drive_pub = rospy.Publisher('right_drive_controller/command', Float64)
        
        rospy.Subscriber("joy", Joy, joystick_callback)
        
        rospy.spin()
        
if __name__ == '__main__':
        main()
