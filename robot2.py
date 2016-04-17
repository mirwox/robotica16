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

	ser.write("SetLDSRotation On\n")
	data = read ()
	print(data)
	time.sleep(4)

	ser.write("GetLDSScan\n")
	time.sleep(4)
	data1 = read()
	time.sleep(0.1)
	data2 = read()
	data = data1 + data2
	data = data.split(",")[4:len(data.split(","))-1]
	dist = list()
	a = 0
	for i in data:
		if a == 3:
			a = 0
		if a == 0:
			dist.append(int(i))
		a += 1

	dir = [8050]*360
	for i in range(len(dist)):
		if dist[i] > 599 and dist[i] < 1401:
			dir[i] = dist[i]

	print(dir.index(min(dir)))
	while dir.index(min(dir)) <= 335 or dir.index(min(dir)) >= 355:


		ser.write("SetMotor 25 -25 150\n")

		ser.write("GetLDSScan\n")
		time.sleep(0.8)
		data1 = read()
		time.sleep(0.8)
		data2 = read()
		data = data1 + data2
		data = data.split(",")[4:len(data.split(","))-1]
		dist = list()
		a = 0
		for i in data:
			if a == 3:
				a = 0
			if a == 0:
				dist.append(int(i))
			a += 1

		dir = [8050]*360
		for i in range(len(dist)):
			if dist[i] > 599 and dist[i] < 1401:
				dir[i] = dist[i]

		print(dir.index(min(dir)))


		
		while dir.index(min(dir)) >= 335:
			ser.write("SetMotor 100 100 150\n")
			ser.write("GetLDSScan\n")
			time.sleep(0.8)
			data1 = read()
			time.sleep(0.8)
			data2 = read()
			data = data1 + data2
			data = data.split(",")[4:len(data.split(","))-1]
			dist = list()
			a = 0
			for i in data:
				if a == 3:
					a = 0
				if a == 0:
					dist.append(int(i))
				a += 1

			dir = [8050]*360
			for i in range(len(dist)):
				if dist[i] > 599 and dist[i] < 1401:
					dir[i] = dist[i]

			print(dir.index(min(dir)))	

		dir[dir.index(min(dir))] = 8050





	ser.close()


if __name__ == "__main__":
	main()
