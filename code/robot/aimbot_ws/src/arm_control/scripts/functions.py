
from dynamixel_sdk import * 


def setup(packetHandler, portHandler, id):
	ADDR_MX_TORQUE_ENABLE = 24               # Control table address is different in Dynamixel model
	ADDR_MX_CW_ANGLE_LIMIT = 6
	ADDR_MX_CCW_ANGLE_LIMIT = 8
	ADDR_MX_CW_LIMIT = 6
	ADDR_MX_CCW_LIMIT = 8
	ADDR_MX_MAX_TORQUE = 14
	ADDR_MX_TORQUE_LIMIT = 34

	TORQUE_ENABLE = 0                 # Value for enabling the torque
	CW_ANGLE_LIMIT = 0
	CCW_ANGLE_LIMIT = 0
	MAX_TORQUE = 1023
	TORQUE_LIMIT = 1023



	
	# Dynamixel Torque
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
	if dxl_comm_result != COMM_SUCCESS:
	    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
	elif dxl_error != 0:
	    print("%s" % packetHandler.getRxPacketError(dxl_error))
	else:
	    print("Dynamixel has been successfully connected")

	# Set max torque
	dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id, ADDR_MX_MAX_TORQUE, MAX_TORQUE)
	if dxl_comm_result != COMM_SUCCESS:
	    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
	elif dxl_error != 0:
	    print("%s" % packetHandler.getRxPacketError(dxl_error))
	else:
	    print("Dynamixel has been successfully connected")

	# Set torque limit
	dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id, ADDR_MX_TORQUE_LIMIT, TORQUE_LIMIT)
	if dxl_comm_result != COMM_SUCCESS:
	    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
	elif dxl_error != 0:
	    print("%s" % packetHandler.getRxPacketError(dxl_error))
	else:
	    print("Dynamixel has been successfully connected")



	# Set CW limit
	dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id, ADDR_MX_CW_LIMIT, CW_ANGLE_LIMIT)
	if dxl_comm_result != COMM_SUCCESS:
	    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
	elif dxl_error != 0:
	    print("%s" % packetHandler.getRxPacketError(dxl_error))
	else:
	    print("Dynamixel has been successfully connected")


	# Set CCW limit
	dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id, ADDR_MX_CCW_LIMIT, CCW_ANGLE_LIMIT)
	if dxl_comm_result != COMM_SUCCESS:
	    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
	elif dxl_error != 0:
	    print("%s" % packetHandler.getRxPacketError(dxl_error))
	else:
	    print("Dynamixel has been successfully connected")




def read_position(packetHandler, portHandler):
	servo_ids = [1, 2, 3, 4, 5]

	ADDR_MX_PRESENT_POSITION = 36


	position = []
	
	for current_id in servo_ids:
		# Read present position
		dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, current_id, ADDR_MX_PRESENT_POSITION)
		if dxl_comm_result != COMM_SUCCESS:
		    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
		    print("%s" % packetHandler.getRxPacketError(dxl_error))

		pos = (float(dxl_present_position) / 1023.0) * 300 - 150
		print(pos)
		time.sleep(0.1)

		position.append(pos)

	return position


#position = array of angles
def save_position(file_name, position):
	servo_ids = [1, 2, 3, 4, 5]


	f = open(file_name, "a")
	
	for i in range(0, len(servo_ids)):
		f.write(str(position[i]) + " ")

	f.write("\n")
	print(position)
	f.close()
	



def read_path(file_name):
	f = open(file_name, "r")

	path = []
	done = False
	while True:
		data = f.readline()
		data = str.split(data)
		if data == []:
			break
		for i in range(0, len(data)):
			data[i] = float(data[i])
		path.append(data)
	
	#print(path)
	f.close()
	
	return path

def follow_path(packetHandler, portHandler, path):
	speed = 55
	ADDR_GOAL_POSITION = 30
	ADDR_MOVING_SPEED = 32
	servo_ids = [1, 2, 3, 4, 5]

	for i in path:
		print(i)
		
		j = 0
		for current_id in servo_ids:
			
			angle_mem_value = (i[j] + 150)/300*1023
			
			# set moving speed
			dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler,current_id, ADDR_MOVING_SPEED, int(speed))
			if dxl_comm_result != COMM_SUCCESS:
			    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
			elif dxl_error != 0:
			    print("%s" % packetHandler.getRxPacketError(dxl_error))
			else:
			    print("Dynamixel has been successfully connected")



			#set goal position
			dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, current_id, ADDR_GOAL_POSITION, int(angle_mem_value))
			if dxl_comm_result != COMM_SUCCESS:
			    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
			elif dxl_error != 0:
			    print("%s" % packetHandler.getRxPacketError(dxl_error))
			else:
			    print("Dynamixel has been successfully connected")

			print(i[j])
			j = j + 1

			



		time.sleep(4)
		
	return





def set_torque_limit_arm(packetHandler, portHandler, torque_limit):
	ADDR_TORQUE_LIMIT = 34	
	servo_ids = [1, 2, 3, 4, 5]

	
	j = 0
	for current_id in servo_ids:
		
		dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, current_id, ADDR_TORQUE_LIMIT, torque_limit[j])
		if dxl_comm_result != COMM_SUCCESS:
		    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
		    print("%s" % packetHandler.getRxPacketError(dxl_error))
		else:
		    print("Dynamixel has been successfully connected")
		
		j = j + 1





