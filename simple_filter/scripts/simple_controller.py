#!/usr/bin/env python

import rospy
import cv2
from simple_filter.msg import VelocitySimple

class Teleop(object):
    def __init__(self):
        rospy.init_node('teleop')
        self.pub = rospy.Publisher('/simple_vel', VelocitySimple, queue_size=10)
        cv2.namedWindow('main_window')

    def run(self):
        while not rospy.is_shutdown():
            key = cv2.waitKey(10)
            if key == 65363:
                self.pub.publish(VelocitySimple(south_to_north_velocity=1))
            elif key == 65361:
                self.pub.publish(VelocitySimple(south_to_north_velocity=-1))

if __name__ == '__main__':
    node = Teleop()
    node.run()