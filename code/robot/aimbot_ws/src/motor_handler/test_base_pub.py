

import rospy
import time
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState

pub = rospy.Publisher('motor_data', JointState, queue_size=10)
rospy.init_node('test_pub_motor_data')
r = rospy.Rate(1)

while not rospy.is_shutdown():
	joint = JointState()
	joint.header.stamp = rospy.Time.now()
	joint.position = []
	joint.velocity = [0, 0]
	joint.name = ["6", "7"]
	
	pub.publish(joint)
	
	r.sleep()
