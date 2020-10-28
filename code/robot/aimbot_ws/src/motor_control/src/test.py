#!/usr/bin/env python3

import rospy
#import dynamixel_functions as dynamixel
from dynamixel_sdk import *
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from message_filters import ApproximateTimeSynchronizer, Subscriber
# Setup
DEVICENAME = "USB_TYY_CONNECTION__THING"
BAUDRATE = 115200
PROTOCOL_VERSION = 1.0
# Control table
# See emanual for info
ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_PRESENT_POSIITON = 36

# Byte lengths
LEN_GOAL_POSITION = 4

# Initial values
dynamixel_arm_ids = [1, 2, 3, 4, 5]
dynamixel_base_ids = [6, 7]


portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)


# Establish connection to dynamixels
#if portHandler.openPort():
#    rospy.loginfo("Succeeded to open Dynamixel port")
#else:
#    rospy.logerror("Failed to open Dynamixel port")
#    rospy.logerror("Aborting...")
#    quit()

#if portHandler.setBaudRate(BAUDRATE):
#    loginfo("Changed baudrate to %s", BAUDRATE)
#else:
#    logwarn("Failed to change baudrate to %s", BAUDRATE)
#    logwarn("Continuing...")

#group_num = dynamixel.
# Get data from arm
# Get data from base
# Send data to dynamixels
def send_to_motors(base):
    print("Arm %s", base.position)
    #print("Base: %s", base.data)
# Read data from dynamixels
# Publish arm feedback
# Pubish base feedback

def callback(data):
    rospy.loginfo("I heard %s", data.data)

def listener():
    # Initialization
    rospy.init_node('motor_node')
    base_sub = Subscriber("base_motors", JointState)
    ats = ApproximateTimeSynchronizer([base_sub], queue_size=2, slop=0.1)
    ats.registerCallback(send_to_motors)
    rospy.spin()
if __name__ == '__main__':
    listener()
