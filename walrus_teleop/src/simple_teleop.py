#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy

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

        mod_front_left = data.buttons[4];
        mod_front_right = data.buttons[5];
        mod_back_left = data.buttons[6];
        mod_back_right = data.buttons[7];
        if mod_front_left or mod_front_right or mod_back_left or mod_back_right:
                joint_delta = left/2; #rad/s
                joint_state[0] += mod_front_left*joint_delta*dt
                joint_state[1] += mod_front_right*joint_delta*dt
                joint_state[2] += mod_back_left*joint_delta*dt
                joint_state[3] += mod_back_right*joint_delta*dt
                left_drive_pub.publish(0)
                right_drive_pub.publish(0)
        else:
                left_drive_pub.publish(-10*left)
                right_drive_pub.publish(-10*right)

        if reset_joints:
                joint_state[0:4] = [0, 0, 0, 0]
                
        if normalize_joints:
                avg = sum(joint_state)/len(joint_state);
                joint_state[0:4] = [avg, avg, avg, avg]

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
