#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import JointState

def get_ros_message(speed):
    msg = JointState()
    msg.header.stamp = rospy.get_rostime()
    msg.name = ["right_wheel", "left_wheel"]
    msg.velocity = speed
    return msg


if __name__ == '__main__':
    pub = rospy.Publisher('motor_control', JointState, queue_size=1)
    rospy.init_node('move_control2', anonymous=True)
    rate = rospy.Rate(10)
    speed_msg = get_ros_message([0,0])
    pub.publish(speed_msg)
    
    

			
	








