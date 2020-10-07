from pyax12.connection import Connection
import time

import functions


# Connect to the serial port
serial_connection = Connection(port="/dev/ttyAMA0", rpi_gpio=True)
file_name = "test1.txt"


path = functions.read_path(serial_connection, file_name)
functions.follow_path(serial_connection, path)

# Close the serial connection
serial_connection.close()
