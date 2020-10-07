from pyax12.connection import Connection as conn

serial_connection = conn(port="/dev/ttyAMA0", rpi_gpio=True)


ids_available = serial_connection.scan()

for dynamixel_id in ids_available:
	print(dynamixel_id)

serial_connection.close()



