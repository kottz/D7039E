#!/usr/bin/python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Ryu Woon Jung (Leon)

#
# *********     Read and Write Example      *********
#
#
# Available DXL model on this example : All models using Protocol 1.0
# This example is tested with a DXL MX-28, and an USB2DYNAMIXEL
# Be sure that DXL MX properties are already set as %% ID : 1 / Baudnum : 34 (Baudrate : 57600)
#

#! /usr/bin/env python3
import rospy
#import dynamixel_functions as dynamixel
from dynamixel_sdk import *
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from std_msgs.msg import String
#from message_filters import ApproximateTimeSynchronizer, Subscriber
import signal
import sys


import os


if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library





# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 6               # Dynamixel ID : 1

BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"



# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()





















ADDR_GOAL_POSITION = 30
ADDR_MOVING_SPEED = 32



def set_position(id_motor, position):
	


	#print("Setting position to " + str(position) + " for id " + str(id_motor))
	
	angle_mem_value = (position + 150)/300*1023
	#set goal position
	dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, int(id_motor), ADDR_GOAL_POSITION, int(angle_mem_value))
	if dxl_comm_result != COMM_SUCCESS:
		print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
	elif dxl_error != 0:
		print("%s" % packetHandler.getRxPacketError(dxl_error))
	else:
		print("Position set to: " + str(position) + " for motor id: " + str(id_motor))




def set_speed(id_motor, velocity):
	#print("Setting velocity to " + str(velocity) + " for id " + str(id_motor))

	velocity_mem_value = (velocity)/100*1023
	# Set moving speed
	dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, int(id_motor), ADDR_MOVING_SPEED, int(velocity_mem_value))
	if dxl_comm_result != COMM_SUCCESS:
		print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
	elif dxl_error != 0:
		print("%s" % packetHandler.getRxPacketError(dxl_error))
	else:
		print("Velocity set to: " + str(velocity) + " for motor id: " + str(id_motor))






def send_to_motors(data):
	
	#print(data)
	#send speed and position data to motors
	for i in range(0, len(data.name)):
		try:
			set_speed(int(data.name[i]), int(data.velocity[i]))		
		except:
			pass


		try:
			set_position(int(data.name[i]), int(data.position[i]))
		except:
			pass




def listener(): 	# Set up subscriber to ros-node
    # Initialization
    rospy.init_node('motor_handler')
    base_sub = rospy.Subscriber("motor_data", JointState, send_to_motors)
    #ats = ApproximateTimeSynchronizer([base_sub], queue_size=2, slop=0.1)
    #ats.registerCallback(send_to_motors)
    rospy.spin()
#####################################    
 


    
######################################    
if __name__ == '__main__':
    listener()








def at_exit():
	# Close port
	portHandler.closePort()

import atexit
atexit.register(at_exit)


