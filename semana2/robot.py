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

    ser.write("GetDigitalSensors\n")
    time.sleep(1)
    data = read()
    print(data)

    ser.write("GetAnalogSensors\n")
    time.sleep(1)
    data = read()
    print(data)

    ser.write("SetMotor -100 100 80\n")
    time.sleep(1)
    data = read()
    print(data)

    ser.write("SetMotor 100 -100 80 \n")
    time.sleep(1)
    data = read()
    print(data)

    ser.write("SetLDSRotation On\n")
    time.sleep(4)
    data = read()
    print(data)

    ser.write("GetLDSScan\n")
    time.sleep(2)
    data = read()
    print(data)


    ser.write("SetLDSRotation Off\n")
    time.sleep(1)
    data = read()
    print(data)

    ser.close()


if __name__ == "__main__":
    main()
