import serial
import time
import RPi.GPIO as GPIO


GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
GPIO.setwarnings(False)

port=serial.Serial("/dev/ttyAMA0", baudrate=1000000, timeout=1.0)


GPIO.output(18, GPIO.HIGH)
print("sending")
port.write(bytearray.fromhex("FF FF 02 04 03 04 22 D0"))

time.sleep(0.000400)
GPIO.output(18, GPIO.LOW)



done = False
s = -255-255

x = port.read()
while done != True:
	print(x)
	#check checksum
	ch = 255 - s
	if ch == int.from_bytes(x, "big"):
		done = True
	#print(ch)
	s = s + int.from_bytes(x, "big")
	
	if x == b'':
		print("no valid checksum")
		done = True
	
	x = port.read()
	
	



#time.sleep(1)
	




