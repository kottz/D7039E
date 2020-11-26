#!/usr/bin/env python3

import threading
import rospy
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
import time
import pandas


SWIVEL_MOTOR 	   = "1"
FIRST_JOINT_MOTOR  = "2"
SECOND_JOINT_MOTOR = "3"
THIRD_JOINT_MOTOR  = "4"
GRIPPER_MOTOR 	   = "5"

# Calculates appropriate servo setting from angle.
# We have a few different options: 1) Convert angle here
                                  #2) Create custom arm and base motor messages. Process them in different queues.
                                  #3) Define position as angle.
def convert_angles(angles):
    pos = []
    for a in angles:
        if a > 150 or a < -150:
            print("position is out of range")
        pos.append((a + 150)/300*1023)
        
    return pos
	
# Calculates appropriate servo setting from speed list. Same discussion as above
def convert_speeds(speed):
    vel = []
    for s in speed:
        dir_value = 0
        #if speed is negative (it should be in wheel mode) "switch" direction
        if s < 0:
            s = -s
            dir_value = 1024
        velocity_mem_value = s/100*1023     
        vel.append(velocity_mem_value + dir_value)
    #print("speed: " + str(vel)) 
    return vel
			
class ArmControl:
 
    def __init__(self):
        self.arm_publisher = rospy.Publisher('motor_control', JointState, queue_size=5)

    # Service to pick up a piece
    def pickup(self, req):
        # Different motor angles during pickup procedure
        file_path = "src/arm_control/src/"
        path = pandas.read_csv(file_path + "traj_test.csv")
        vel = pandas.read_csv(file_path + "velocity_test.csv")


        q = []
        velocities = []

        #load positions and velocities for the trajectory
        for i in range(0, len(path.iloc[:,0].tolist())):
            t = path.iloc[i,:].tolist()
            q.append(t)	

        for i in range(0, len(vel.iloc[:,0].tolist())):
            t = vel.iloc[i,:].tolist()
            velocities.append(t)	
            
        path = q

            #        path = [[0, 0, 90, 0], [0, 0, 90, 0, 0], [0, 57, 51, 2, 0], [0, 57, 51, 2, 100], [0, 0, 90, 0, 100]]
            
        
        for i in range(0, len(path)):
            p = path[i]
            v = velocities[i]
            v = [abs(vv)/10 for vv in velocities[i]]
            print("speed " + str(v))
            print("angles " + str(p))
            joint = JointState()
            angles = p
            #print(angles)
            joint.header.stamp = rospy.Time.now()
            joint.position = convert_angles(angles)
            #joint.velocity = convert_speeds(v)
            joint.velocity = convert_speeds([50, 50, 50, 50, 70])
            joint.name = [SWIVEL_MOTOR, FIRST_JOINT_MOTOR, SECOND_JOINT_MOTOR, THIRD_JOINT_MOTOR, GRIPPER_MOTOR]
            self.arm_publisher.publish(joint)
            time.sleep(0.05)
        return ()

    # Service to return to line follow position (does not touch gripper since it might be in use)
    def reset_arm(self, req):
        joint = JointState()
        joint.header.stamp = rospy.Time.now()
        joint.position = convert_angles([0, 0, 90, 0])
        joint.velocity = convert_speeds([10, 10, 10, 10])
        joint.name = [SWIVEL_MOTOR, FIRST_JOINT_MOTOR, SECOND_JOINT_MOTOR, THIRD_JOINT_MOTOR]
        rospy.logwarn("In arm reset")
        self.arm_publisher.publish(joint)
        return ()

    # TODO: start_listeners and init_services should not be class methods.
    def init_services(self):
        rospy.Service('pickup', Empty, self.pickup)
        rospy.Service('reset_arm', Empty, self.reset_arm)
           


if __name__ == '__main__':
	
	rospy.init_node('arm_control', anonymous=True)
	
	a = ArmControl()
	a.init_services()
	print("reached inits ok")

	rospy.spin()


