#!/usr/bin/env python

""" A very simple simulator for a robot in 1d with forward
    and rear facing range sensor """

import rospy
from numpy.random import randn
from simple_filter.msg import LaserSimple, VelocitySimple , OdometrySimple
import numpy as np
from std_msgs.msg import Float64

class SimpleWorldNode(object):
    def __init__(self):
        rospy.init_node('simple_world')
        walls = rospy.get_param('~walls')
        self.world = WorldModel(walls=walls)
        self.pub = rospy.Publisher('/simple_scan', LaserSimple, queue_size=10)
        self.pub_pos = rospy.Publisher('/true_position', Float64, queue_size=10)
        self.pub_odom = rospy.Publisher('/simple_odom', OdometrySimple, queue_size=10)
        self.next_velocity = None
        rospy.Subscriber('/simple_vel', VelocitySimple, self.process_simple_vel)

    def run(self):
        r = rospy.Rate(1./self.world.dt)
        while not rospy.is_shutdown():
            if self.next_velocity == None:
                sensation = self.world.get_sensation()
            else:
                sensation = self.world.do_action(self.next_velocity)
                self.next_velocity = None
            self.pub.publish(sensation)
            self.pub_pos.publish(self.world.position)
            self.pub_odom.publish(OdometrySimple(south_to_north_position=self.world.odom_position))
            r.sleep()

    def process_simple_vel(self, msg):
        self.next_velocity = msg.south_to_north_velocity

class WorldModel(object):
    def __init__(self, noise_rate=.05, odom_noise_rate=.1, walls=None):
        if walls == None:
            self.walls =[]
        else:
            self.walls = walls
        self.position = randn()*0.2+1.5
        self.odom_position = 0.0
        self.odom_noise_rate = odom_noise_rate
        self.noise_rate = noise_rate
        self.dt = .2

    def add_wall(self, wall_position):
        self.walls.append(wall_position)

    def get_closest_obstacle(self, position, direction):
        if direction == -1:
            positions = [(position - w, idx) for idx, w in enumerate(self.walls) if position - w >= 0]
            if len(positions) == 0:
                return None
            min_idx = np.argmin([p[0] for p in positions])
            return self.walls[positions[min_idx][1]]
        else:
            positions = [(w - position, idx) for idx, w in enumerate(self.walls) if w - position >= 0]
            if len(positions) == 0:
                return None
            min_idx = np.argmin([p[0] for p in positions])
            return self.walls[positions[min_idx][1]]

    def get_sensation(self):
        closest_north = self.get_closest_obstacle(self.position, 1)
        closest_south = self.get_closest_obstacle(self.position, -1)
        if closest_north == None:
            north_laser_reading = 0.0
        else:
            north_laser_reading = (closest_north - self.position) + self.noise_rate*randn()
        if closest_south == None:
            south_laser_reading = 0.0
        else:
            south_laser_reading = (self.position - closest_south) + self.noise_rate*randn()

        return LaserSimple(south_laser= south_laser_reading,
                           north_laser= north_laser_reading)

    def do_action(self, velocity):
        self.position += velocity*self.dt
        self.odom_position += velocity*self.dt + self.odom_noise_rate*randn()
        return self.get_sensation()

if __name__ == '__main__':
    node = SimpleWorldNode()
    node.run()