from pyax12.connection import Connection
import time

s=100

# Connect to the serial port
serial_connection = Connection(port="/dev/ttyAMA0", rpi_gpio=True)

dynamixel_id1 = 6
dynamixel_id2 = 7

angle1 = 0
angle2= -0
# Go to 0°
for i in range(0,10):
    serial_connection.goto(dynamixel_id1, angle1, speed=s, degrees=True)

    print(i)
    serial_connection.goto(dynamixel_id2, angle1, speed=s, degrees=True)


    
    time.sleep(4)
    
# Close the serial connection
serial_connection.close()
