from pyax12.connection import Connection
import time

servo_ids = [1, 2, 3, 4, 5]



def read_position(serial_connection):
	position = []
	
	for current_id in servo_ids:
		position.append(serial_connection.get_present_position(current_id, degrees = True))

	return position


#position = array of angles
def save_position(serial_connection, file_name, position):
	f = open(file_name, "a")
	
	for i in range(0, len(servo_ids)):
		f.write(str(position[i]) + " ")

	f.write("\n")
	print(position)
	f.close()
	
def read_path(serial_connection, file_name):
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

def follow_path(serial_connection, path):
	s = 35

	for i in path:
		print(i)
		
		j = 0
		for current_id in servo_ids:
			serial_connection.goto(current_id, i[j], speed=s, degrees=True)
			j = j + 1

		#time.sleep(1.5)
		
		max_error = 10;
		dynamixels_done = False
		while dynamixels_done == False:
			d = 0 #error distance
			j = 0 #loop variable
			for current_id in servo_ids:
				d = d + abs((serial_connection.get_present_position(current_id, degrees=True)-i[j]))
				time.sleep(0.1)
				j = j + 1 
			

			if d < max_error:
				break
			#if serial_connection.is_moving(1) == False and serial_connection.is_moving(2) == False and serial_connection.is_moving(3) == False and serial_connection.is_moving(4) == False:
			#	break
			time.sleep(0.1)
	return




