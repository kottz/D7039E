#!/usr/bin/python3
import rospy
import time
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from arm_control.srv import *
import functions


#rospy.init_node('arm_pickup')

#pub = rospy.Publisher('motor_data', JointState, queue_size=10)


	
#file_name = "pick_from_factory.txt"


#path = functions.read_path(file_name)
def myfun(req):
    print(req.command)
    if req.command == "a_line_following":
        joint = JointState()
        joint.header.stamp = rospy.Time.now()
        joint.position = [10, 10, 90, 0, 0]
        joint.velocity = [10, 10, 10, 10, 10]
        joint.name = ["1", "2", "3", "4", "5"]
        return (1, joint.name, joint.position, joint.velocity)
    else:
        rospy.logwarn("Failed")
        return (0, [0,0], [0,0], [0,0])

def ignore():
    joint = JointState()
    for i in path:
        angles = i
        print(angles)		
        joint.header.stamp = rospy.Time.now()
        joint.position = angles
        joint.velocity = [10, 10, 10, 10, 10]
        joint.name = ["1", "2", "3", "4", "5"]
       # pub.publish(joint)
        #time.sleep(3)
    print("B4 return") 
    return (1,1,1,1)

rospy.init_node('arm_pickup_service')
s = rospy.Service('arm_joints_srv', ArmJoints, myfun)
rospy.spin()

