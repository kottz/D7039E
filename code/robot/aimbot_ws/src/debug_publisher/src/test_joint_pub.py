

import rospy
import time
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState

pub = rospy.Publisher('motor_data', JointState, queue_size=10)
rospy.init_node('node123')
r = rospy.Rate(1)

while not rospy.is_shutdown():
	joint = JointState()
	joint.header.stamp = rospy.Time.now()
	#joint.position = [0, 0, 90, 0, 0]
	joint.velocity = [20, 20]
	joint.name = ["6", "7"]
	
	pub.publish(joint)
	
	r.sleep()
