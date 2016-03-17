#! env python
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Vector3

v = 5   # Velocidade angular
w = 10  # Velocidade linear

if __name__ == "__main__":
    rospy.init_node("roda.py")
    vel = Twist(Vector3(v,0,0), Vector3(0,0,w))
    pub = rospy.Publisher("cmd_vel", Twist)

    try:
        while not rospy.is_shutdown():
            pub.publish(vel)
            rospy.sleep(1.0)
    except rospy.ROSInterruptedException:
        print("Ocorreu uma exceção com o rospy")
