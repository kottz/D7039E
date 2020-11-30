#!/usr/bin/env python3

import rospy
from arm_control.srv import *

rospy.init_node('robot_main')


def start():
    rospy.wait_for_service("arm_joints_srv")
    try:
        arm_joints = rospy.ServiceProxy("arm_joints_srv",ArmJoints)
        resp = arm_joints("a_line_following")
        print(resp.velocity)
    except rospy.ServiceException as e:
        print("error when calling arm_joints_srv: %s" %e)
        sys.exit(1)

start()    















