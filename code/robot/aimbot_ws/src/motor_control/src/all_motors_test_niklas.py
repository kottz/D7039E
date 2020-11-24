#!/usr/bin/python

import rospy

from dynamixel_sdk import *
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from std_msgs.msg import String
#from message_filters import ApproximateTimeSynchronizer, Subscriber
import signal
import sys


SWIVEL_MOTOR 	   = 1
FIRST_JOINT_MOTOR  = 2
SECOND_JOINT_MOTOR = 3
THIRD_JOINT_MOTOR  = 4
GRIPPER_MOTOR 	   = 5
LEFT_WHEEL_MOTOR   = 6
RIGHT_WHEEL_MOTOR  = 7

# Protocol version

PROTOCOL_VERSION = 1.0  # See which protocol version is used in the Dynamixel

# Default setting

DXL_ID = 6  # Dynamixel ID
BAUDRATE = 57600  # Dynamixel default baudrate : 57600
DEVICENAME = '/dev/ttyUSB0'  # Check which port is being used on your controller

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

TORQUE_ENABLE = 0  # Value for enabling the torque
CW_ANGLE_LIMIT = 0
CCW_ANGLE_LIMIT = 0
MOVING_SPEED = 1023
MAX_TORQUE = 1023
TORQUE_LIMIT = 1023
MAX_TORQUE = 1023

ADDR_GOAL_POSITION = 30
ADDR_MOVING_SPEED = 32

def signal_handler(signal, frame):
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class Motor:	#Setup the motor arguments
    motor_flip = False
    speed_offset = 1024
    wheel_mode = False
    def __exit__(self):
      self.set_speed(0)

    def __init__(self, motor_id, motor_flip, wheel_mode):	#Setup the write to motor func
        self.motor_id = motor_id
        self.motor_flip = motor_flip
        self.wheel_mode = wheel_mode
        if self.wheel_mode:
            self.setup_wheel()
        else:
            self.setup_joint()

    def set_speed(self, speed):	#The write to motor func
        if self.motor_flip:		#Checks which motor to write to and apply offset
            speed += 1024

        (dxl_comm_result, dxl_error) = \
            packetHandler.write2ByteTxRx(portHandler, self.motor_id,
                ADDR_MOVING_SPEED, int(speed))
        if dxl_comm_result != COMM_SUCCESS:
            print('%s' \
                % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print('%s' % packetHandler.getRxPacketError(dxl_error))
        else:
            print('Dynamixel speed has been set')

    def set_position(self, position):
            rospy.logwarn("Setting position to " + str(position) + " for id " + str(self.motor_id))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, int(self.motor_id), ADDR_GOAL_POSITION, int(position))
            if dxl_comm_result != COMM_SUCCESS:
                rospy.logwarn("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                rospy.logwarn("%s" % packetHandler.getRxPacketError(dxl_error))
            else:
                rospy.logwarn("Position set to: " + str(position) + " for motor id: " + str(self.motor_id))
		
    def set_motors(self, speed, position):
        self.set_speed(speed)
        self.set_position(position)
      
    def setup_joint(self):

        cw_limit = 0
        ccw_limit = 1023

        max_torque = 300

        if self.motor_id == 5:
            max_torque = 100

        # Set torque limit

        (dxl_comm_result, dxl_error) = \
            packetHandler.write2ByteTxRx(portHandler, self.motor_id,
                ADDR_MX_TORQUE_LIMIT, max_torque)
        if dxl_comm_result != COMM_SUCCESS:
            print('%s' % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print('%s' % packetHandler.getRxPacketError(dxl_error))
        else:
            print('Dynamixel has been successfully connected')
 
        #If these two values below are 0 then the motor will be in wheel mode
        # Set CW limit
        (dxl_comm_result, dxl_error) = \
            packetHandler.write2ByteTxRx(portHandler, self.motor_id,
                ADDR_MX_CW_LIMIT, cw_limit)
        if dxl_comm_result != COMM_SUCCESS:
            print('%s' % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print('%s' % packetHandler.getRxPacketError(dxl_error))
        else:
            print('Dynamixel CW Limit set')

        # Set CCW limit

        (dxl_comm_result, dxl_error) = \
            packetHandler.write2ByteTxRx(portHandler, self.motor_id,
                ADDR_MX_CCW_LIMIT, ccw_limit)
        if dxl_comm_result != COMM_SUCCESS:
            print('%s' % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print('%s' % packetHandler.getRxPacketError(dxl_error))
        else:
            print('Dynamixel CCW limit set')








    def setup_wheel(self):

        # Dynamixel Torque

        (dxl_comm_result, dxl_error) = \
            packetHandler.write1ByteTxRx(portHandler, self.motor_id,
                ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print('%s' % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print('%s' % packetHandler.getRxPacketError(dxl_error))
        else:
            print('Dynamixel has been successfully connected')

        # Set max torque

        (dxl_comm_result, dxl_error) = \
            packetHandler.write2ByteTxRx(portHandler, self.motor_id,
                ADDR_MX_MAX_TORQUE, MAX_TORQUE)
        if dxl_comm_result != COMM_SUCCESS:
            print('%s' % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print('%s' % packetHandler.getRxPacketError(dxl_error))
        else:
            print('Dynamixel has been successfully connected')

        # Set torque limit

        (dxl_comm_result, dxl_error) = \
            packetHandler.write2ByteTxRx(portHandler, self.motor_id,
                ADDR_MX_TORQUE_LIMIT, TORQUE_LIMIT)
        if dxl_comm_result != COMM_SUCCESS:
            print('%s' % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print('%s' % packetHandler.getRxPacketError(dxl_error))
        else:
            print('Dynamixel has been successfully connected')
        
        #If these two values below are 0 then the motor will be in wheel mode
        # Set CW limit
        (dxl_comm_result, dxl_error) = \
            packetHandler.write2ByteTxRx(portHandler, self.motor_id,
                ADDR_MX_CW_LIMIT, CW_ANGLE_LIMIT)
        if dxl_comm_result != COMM_SUCCESS:
            print('%s' % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print('%s' % packetHandler.getRxPacketError(dxl_error))
        else:
            print('Dynamixel CW Limit set')

        # Set CCW limit

        (dxl_comm_result, dxl_error) = \
            packetHandler.write2ByteTxRx(portHandler, self.motor_id,
                ADDR_MX_CCW_LIMIT, CCW_ANGLE_LIMIT)
        if dxl_comm_result != COMM_SUCCESS:
            print('%s' % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print('%s' % packetHandler.getRxPacketError(dxl_error))
        else:
            print('Dynamixel CCW limit set')

   # def _set_motor(self, position, velocity):
    #    # We want this to be generic, to control both arm and base motors

        
        
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



# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows




portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)


# Initialize GroupSyncWrite instance
ADDR_AX_GOAL_POSITION = 30 # speed = 32
LEN_SPEED_POSITION = 4
groupSyncWriteVelPos = GroupSyncWrite(portHandler, packetHandler, ADDR_AX_GOAL_POSITION, LEN_SPEED_POSITION)


ADDR_AX_SPEED = 32
LEN_SPEED = 2
groupSyncWriteVel = GroupSyncWrite(portHandler, packetHandler, ADDR_AX_SPEED, LEN_SPEED)



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


def sync_speed(ids, velocities):
    print("SYNC WRITE SPEED")
    
    ids = [int(i) for i in ids] #convert to int
    #velocities = [int(v) for v in velocities] 
    #print(velocities)
    for i in range(0, len(ids)):
        #Allocate params into byte array
        param = []
        

        #very bad, just for testing
        if ids[i] == 7: #"flip" motor 7
            velocities[i] += 1024

        param.append(DXL_LOBYTE(int(velocities[i])))
        param.append(DXL_HIBYTE(int(velocities[i])))
       
       
        #print(param)
 
        
        # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWriteVel.addParam(ids[i], param)
        if dxl_addparam_result != True:
           print("[ID:%03d] groupSyncWrite addparam failed" % i)
           quit()



    # Syncwrite goal position
    dxl_comm_result = groupSyncWriteVel.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWriteVel.clearParam()
    
    


def sync_speed_position(ids, velocities, positions):
    print("SYNC WRITE SPEED AND POSITION")
    
    ids = [int(i) for i in ids] #convert to int
    #velocities = [int(v) for v in velocities] 
    #print(velocities)
    for i in range(0, len(ids)):
        #Allocate params into byte array
        param = []
      
      
        param.append(DXL_LOBYTE(int(positions[i])))
        param.append(DXL_HIBYTE(int(positions[i])))
        
      
        param.append(DXL_LOBYTE(int(velocities[i])))
        param.append(DXL_HIBYTE(int(velocities[i])))
       
       
        #print(param)
 
        
        # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWriteVelPos.addParam(ids[i], param)
        if dxl_addparam_result != True:
           print("[ID:%03d] groupSyncWrite addparam failed" % i)
           quit()



    # Syncwrite goal position
    dxl_comm_result = groupSyncWriteVelPos.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWriteVelPos.clearParam()
    
    #time.sleep(10)


#m2 = Motor(6, False)
#m1 = Motor(7, True)

motor_dict = {'1': Motor(1, False, False),
                '2': Motor(2, False, False),
                '3': Motor(3, False, False),
                '4': Motor(4, False, False),
                '5': Motor(5, False, False),
                '6': Motor(6, False, True),
                '7': Motor(7, True, True)
            }


# Right now quite a bit of delay is introduced here when line following.
# How to speed up?             
def send_to_motors(data):
    print("callback called")
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    #print(data.velocity[0])
   # print(int(data.velocity[0]))
   # m1.set_motors(int(data.velocity[0]), int(data.position[0]))
   # m2.set_motors(int(data.velocity[1]), int(data.position[1]))
   


    t1 = time.time()
    #Store in array to syncwrite
    velocities = []
    ids = []
    positions = []

    # Update velocities if they were set
    if hasattr(data, 'velocity') & hasattr(data, 'position'):
        
        
        if data.velocity and data.position:
            
            print(data)
            for motor_id, vel, pos in zip(data.name, data.velocity, data.position):
                print("setting motor id {} to vel {} and pos to {}".format(motor_id, str(vel), str(pos)))
                #motor = motor_dict[motor_id]
                #motor.set_speed(vel)
                velocities.append(vel)
                positions.append(pos)
                ids.append(motor_id)
            sync_speed_position(ids, velocities, positions)

            #print(velocities)
            #print(ids)


        elif data.velocity: 
            if hasattr(data, 'velocity'):
                for motor_id, vel in zip(data.name, data.velocity):
                    print("setting motor id {} to velocity {}".format(motor_id, str(vel)))
                    #motor = motor_dict[motor_id]
                    #motor.set_position(pos)
                    velocities.append(vel)
                    ids.append(motor_id)
                #print(ids)
                sync_speed(ids, velocities)
        
        
        elif data.position:
            pass
            #fix later





    else:
        #Update positions if they were sent
        if hasattr(data, 'position'):
            for motor_id, pos in zip(data.name, data.position):
                print("setting motor id {} to pos {}".format(motor_id, str(pos)))
                motor = motor_dict[motor_id]
                motor.set_position(pos)


        print(velocities)
        print(ids)



        #Update positions if they were sent
        if hasattr(data, 'position'):
            for motor_id, pos in zip(data.name, data.position):
                print("setting motor id {} to pos {}".format(motor_id, str(pos)))
                motor = motor_dict[motor_id]
                motor.set_position(pos)



    print("time spent: " + str(time.time() - t1))
        
  


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

