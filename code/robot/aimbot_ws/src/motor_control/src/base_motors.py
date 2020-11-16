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

def signal_handler(signal, frame):
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class Motor:	#Setup the motor arguments

    motor_flip = False
    speed_offset = 1024
    motor_id = None

    def __exit__(self):
      self.set_speed(0)

    def __init__(self, motor_id, motor_flip):	#Setup the write to motor func
        self.motor_id = motor_id
        self.setup(motor_id)
        self.motor_flip = motor_flip

    def set_speed(self, moving_speed):	#The write to motor func
        if self.motor_flip:		#Checks which motor to write to and apply offset
            moving_speed += 1024

        (dxl_comm_result, dxl_error) = \
            packetHandler.write2ByteTxRx(portHandler, self.motor_id,
                ADDR_MX_MOVING_SPEED, moving_speed)
        if dxl_comm_result != COMM_SUCCESS:
            print('%s' \
                % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print('%s' % packetHandler.getRxPacketError(dxl_error))
        else:
            print('Dynamixel has been successfully connected')

    def setup(self, motor_id):

        # Dynamixel Torque

        (dxl_comm_result, dxl_error) = \
            packetHandler.write1ByteTxRx(portHandler, motor_id,
                ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print('%s' % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print('%s' % packetHandler.getRxPacketError(dxl_error))
        else:
            print('Dynamixel has been successfully connected')

        # Set max torque

        (dxl_comm_result, dxl_error) = \
            packetHandler.write2ByteTxRx(portHandler, motor_id,
                ADDR_MX_MAX_TORQUE, MAX_TORQUE)
        if dxl_comm_result != COMM_SUCCESS:
            print('%s' % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print('%s' % packetHandler.getRxPacketError(dxl_error))
        else:
            print('Dynamixel has been successfully connected')

        # Set torque limit

        (dxl_comm_result, dxl_error) = \
            packetHandler.write2ByteTxRx(portHandler, motor_id,
                ADDR_MX_TORQUE_LIMIT, TORQUE_LIMIT)
        if dxl_comm_result != COMM_SUCCESS:
            print('%s' % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print('%s' % packetHandler.getRxPacketError(dxl_error))
        else:
            print('Dynamixel has been successfully connected')

        # Set CW limit

        (dxl_comm_result, dxl_error) = \
            packetHandler.write2ByteTxRx(portHandler, motor_id,
                ADDR_MX_CW_LIMIT, CW_ANGLE_LIMIT)
        if dxl_comm_result != COMM_SUCCESS:
            print('%s' % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print('%s' % packetHandler.getRxPacketError(dxl_error))
        else:
            print('Dynamixel has been successfully connected')

        # Set CCW limit

        (dxl_comm_result, dxl_error) = \
            packetHandler.write2ByteTxRx(portHandler, motor_id,
                ADDR_MX_CCW_LIMIT, CCW_ANGLE_LIMIT)
        if dxl_comm_result != COMM_SUCCESS:
            print('%s' % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print('%s' % packetHandler.getRxPacketError(dxl_error))
        else:
            print('Dynamixel has been successfully connected')


import os
import time

import sys
import tty
import termios
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)


def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


from dynamixel_sdk import *  # Uses Dynamixel SDK library

# Control table address

ADDR_MX_TORQUE_ENABLE = 24  # Control table address is different in Dynamixel model
ADDR_MX_CW_ANGLE_LIMIT = 6
ADDR_MX_CCW_ANGLE_LIMIT = 8
ADDR_MX_CW_LIMIT = 6
ADDR_MX_CCW_LIMIT = 8
ADDR_MX_GOAL_POSITION = 116
ADDR_MX_PRESENT_POSITION = 132
ADDR_MX_MOVING_SPEED = 32
ADDR_MX_MAX_TORQUE = 14
ADDR_MX_TORQUE_LIMIT = 34

# Protocol version

PROTOCOL_VERSION = 1.0  # See which protocol version is used in the Dynamixel

# Default setting

DXL_ID = 6  # Dynamixel ID

BAUDRATE = 57600  # Dynamixel default baudrate : 57600
DEVICENAME = '/dev/ttyUSB0'  # Check which port is being used on your controller

                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE = 0  # Value for enabling the torque
CW_ANGLE_LIMIT = 0
CCW_ANGLE_LIMIT = 0
MOVING_SPEED = 1023
MAX_TORQUE = 1023
TORQUE_LIMIT = 1023
MAX_TORQUE = 1023

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
    print('Succeeded to open the port')
else:
    print('Failed to open the port')
    print('Press any key to terminate...')
    getch()
    quit()

# Set port baudrate

if portHandler.setBaudRate(BAUDRATE):
    print ('Succeeded to change the baudrate')
else:
    print('Failed to change the baudrate')
    print('Press any key to terminate...')
    getch()
    quit()

# setup(6)

#print('hej')

# setup(7)

m2 = Motor(6, False)
m1 = Motor(7, True)


def send_to_motors(data):
  #print("callback called")
  #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
  print(data.velocity[0])
  print(int(data.velocity[0]))
  m1.set_speed(int(data.velocity[0]))
  m2.set_speed(int(data.velocity[1]))


def listener(): 	# Set up subscriber to ros-node
    # Initialization
    rospy.init_node('motor_control')
    base_sub = rospy.Subscriber("motor_control", JointState, send_to_motors)
    #ats = ApproximateTimeSynchronizer([base_sub], queue_size=2, slop=0.1)
   #ats.registerCallback(send_to_motors)
    rospy.spin()
if __name__ == '__main__':
    listener()


# Close port

portHandler.closePort()
