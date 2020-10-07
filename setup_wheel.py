import serial
import time
import RPi.GPIO as GPIO


GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
GPIO.setwarnings(False)

port=serial.Serial("/dev/ttyAMA0", baudrate=57600, timeout=1.0)


GPIO.output(18, GPIO.HIGH)
print("sending")


#port.write(bytearray.fromhex("FF FF FE 04 03 03 01 F6")) #change id
#port.write(bytearray.fromhex("FF FF FE 03 03 03 05 22 ")) 
####################################ID#L##W#ADR#ID#BR#CHK####

port.write(bytearray.fromhex("FF FF 06 05 03 06 00 00 EB")) 
############################("FF#FF#ID#L###I##P#P+1#SUM#))
#L = length, 04 = param +2 
#I = instruction, 03 = write
#P = parameter, adr 8 = CCW ANGLE LIMIT
#P+1 parameter+1, set to 0 
#SUM = checksum

time.sleep(0.000400*100)
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
	
	



