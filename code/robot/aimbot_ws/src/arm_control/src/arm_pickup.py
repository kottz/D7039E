#!/usr/bin/python
import rospy
import time
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState

import functions


rospy.init_node('arm_pickup')

pub = rospy.Publisher('motor_data', JointState, queue_size=10)


	
#file_name = "pick_from_factory.txt"


#path = functions.read_path(file_name)

path = [[0, 0, 90, 0], [0, 0, 90, 0, 0], [0, 57, 51, 2, 0], [0, 57, 51, 2, 100], [0, 0, 90, 0, 100]]

 
joint = JointState()
for i in path:

	angles = i
	print(angles)		
	joint.header.stamp = rospy.Time.now()
	joint.position = angles
	joint.velocity = [10, 10, 10, 10, 10]
	joint.name = ["1", "2", "3", "4", "5"]
	
	pub.publish(joint)


	time.sleep(3)





