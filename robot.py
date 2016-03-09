#!/usr/bin/env python

#!/usr/bin/env python

# Execute sudo pkill -HUP -f tcp_serial_redirect0.py antes de executar este programa
# Para desbloquear a serial



import sys
import os
import time
import signal
import threading
import socket
import codecs
import serial
from os import system

ser  = None

def connect_to_serial():
	global ser
	# connect to serial port
	possible_ports = ['/dev/ttyUSB0','/dev/ttyUSB1']
	ser = serial.Serial()
	ser.baudrate = 115200
	ser.parity   = 'N'
	ser.rtscts   = False
	#ser.xonxoff  = options.xonxoff
	ser.timeout  = 1     # required so that the reader thread can exit
	for p in possible_ports:
		ser.port = p
		try:
			ser.open()
			print "Connected on port: " + p
			return ser
		except:
			time.sleep(1)

def read():
	n = ser.inWaiting()
	data = ser.read(n)
	return data






def main():

	print("Starting connection to Neato on ports")

	ser = connect_to_serial()



	ser.write("TestMode on\n")
	time.sleep(2)
	data = read()
	print(data)

	right = 0
	left = 0
	wall = 0
	s = 0

	while (right+left)/2 > -5:

		ser.write("GetAnalogSensors\n")
		time.sleep(0.1)
		data = read().split(",")
		for i in range(len(data)):
			if "LeftMag" in data[i]:
				left = int(data[i+1])
			if "RightMag" in data[i]:
				right =  int(data[i+1])
			if "Wall" in data[i]:
				wall = int(data[i+1])
			print(wall)

		if wall < 55:
			wheel1 = 75
			wheel2 = 125
		elif wall > 60 and wall < 63:
			wheel1 = 125
			wheel2 = 75
		else:
			wheel1 = 100
			wheel2 = 100


		ser.write("SetMotor {0} {1} 80\n".format(wheel1, wheel2))

		time.sleep(0.1)


	print(wall)

	ser.close()


if __name__ == "__main__":
	main()
