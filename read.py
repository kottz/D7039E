from pyax12.connection import Connection
import time

import functions

# Connect to the serial port
serial_connection = Connection(port="/dev/ttyAMA0", rpi_gpio=True)
file_name = "test1.txt"

pos = functions.read_position(serial_connection)
functions.save_position(serial_connection, file_name, pos)


# Close the serial connection
serial_connection.close()
