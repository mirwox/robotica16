# -*- coding: utf-8 -*-
import socket
import sys


# To be used in conjunction with wireless IMU app
# https://play.google.com/store/apps/details?id=org.zwiener.wimu&hl=en


# Formato do pacote:
# Timestamp [sec], sensorid, x, y, z, sensorid, x, y, z, sensorid, x, y, z
#
# Sensor id:
# 3 - Accelerometer (m/s^2)
# 4 - Gyroscope (rad/s)
# 5 - Magnetometer (micro-Tesla uT)



# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to the port
server_address = ('', 10000)
print >>sys.stderr, 'starting up on %s port %s' % server_address
sock.bind(server_address)

sens = {"3":"Accel m/s^2", "4": "Gyro rad/s", "5": "Mag uT"}

while True:
    print >>sys.stderr, '\nwaiting to receive message'
    data, address = sock.recvfrom(64)

    print 'received %s bytes from %s' % (len(data), address)
    print data
    data  = data.replace(".", "") # Elimina o '.' do final
    p = data.split(",")
    p = [i.strip() for i in p]

    print " Time: {0}  {1} {2:5.3f}, {3:5.3f}, {4:5.3f},  {5} {6:5.3f}, {7:5.3f}, {8:5.3f}".format(p[0], sens[p[1]], float(p[2]), float(p[3]), float(p[4]), sens[p[5]], float(p[6]), float(p[7]), float(p[8]))

    """
        WARNING: Por causa do locale o Python pode converter os valores na função float() usando os pontos como separadores de milhares
    """

    if len(p) > 9:
        print "{0} {1:5.3f} {2:5.3f} {3:5.3f}".format(sens[p[9]], p[10], p[11], p[12])
