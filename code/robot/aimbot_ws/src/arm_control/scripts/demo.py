#!/usr/bin/env python
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

import os
import time
import functions

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

# Control table address
ADDR_MX_TORQUE_ENABLE = 24               # Control table address is different in Dynamixel model
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
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 6               # Dynamixel ID : 1

BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"










TORQUE_ENABLE = 0                 # Value for enabling the torque
CW_ANGLE_LIMIT = 0
CCW_ANGLE_LIMIT = 0

MAX_TORQUE = 1023
TORQUE_LIMIT = 1023








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







functions.setup(packetHandler, portHandler, 6)
print("hej")
functions.setup(packetHandler, portHandler, 7)
drive_time = 6
MOVING_SPEED = 420





print("moving back")
# Set moving speed
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 6, ADDR_MX_MOVING_SPEED, MOVING_SPEED + 1024)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")


# Set moving speed
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 7, ADDR_MX_MOVING_SPEED, MOVING_SPEED)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")


time.sleep(drive_time)


print("stopping base")
# Set moving speed
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 6, ADDR_MX_MOVING_SPEED, 0)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")


# Set moving speed
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 7, ADDR_MX_MOVING_SPEED, 0)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")


time.sleep(drive_time)





print("moving forward")
# Set moving speed
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 6, ADDR_MX_MOVING_SPEED, MOVING_SPEED)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")


# Set moving speed
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 7, ADDR_MX_MOVING_SPEED, MOVING_SPEED + 1024)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")

time.sleep(drive_time)



print("stopping base")
# Set moving speed
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 6, ADDR_MX_MOVING_SPEED, 0)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")


# Set moving speed
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 7, ADDR_MX_MOVING_SPEED, 0)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")









# arm dance
# rotate base
functions.set_torque_limit_arm(packetHandler, portHandler, [1023, 1023, 1023, 1023, 200])


file_name = "pick_up.txt"


path = functions.read_path(file_name)
functions.follow_path(packetHandler, portHandler, path)














print("moving back")
# Set moving speed
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 6, ADDR_MX_MOVING_SPEED, MOVING_SPEED + 1024)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")


# Set moving speed
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 7, ADDR_MX_MOVING_SPEED, MOVING_SPEED)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")


time.sleep(drive_time)

print("stopping base")
# Set moving speed
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 6, ADDR_MX_MOVING_SPEED, 0)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")


# Set moving speed
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 7, ADDR_MX_MOVING_SPEED, 0)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")





file_name = "give_to_ed.txt"


path = functions.read_path(file_name)
functions.follow_path(packetHandler, portHandler, path)



# Close port
portHandler.closePort()













