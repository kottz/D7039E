#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import JointState

def talker():
    pub = rospy.Publisher('arm_motors', JointState, queue_size=10)
    rospy.init_node('pubber', anonymous=True)
    rate = rospy.Rate(10)
    joints = JointState()
    joints.header.stamp = rospy.get_rostime()
    joints.name = ["left wheel", "right_wheel"]
    joints.position = [0, 1]
    while not rospy.is_shutdown():
        testint = 99
        print("pubber")
        joints.header.stamp = rospy.Time.now()
        pub.publish(joints)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
