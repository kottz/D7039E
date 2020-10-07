import serial

serialport = serial.Serial("serial10", baudrate=9600,timeout=3.0)

while True:
	serialport.write("rnSay Someting:")
	rcv = port.read(10)
	serialport.write("rnYou sent:" + repr(rcv))
