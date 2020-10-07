from pyax12.connection import Connection

# Connect to the serial port
serial_connection = Connection(port="/dev/ttyAMA0", rpi_gpio=True)

dynamixel_id = 7

# Print the control table of the specified Dynamixel unit
serial_connection.pretty_print_control_table(dynamixel_id)

# Close the serial connection
serial_connection.close()
