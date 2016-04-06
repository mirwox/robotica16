#!/usr/bin/env python

""" A ROS Node that implements a 1d particle filter """


import matplotlib.pyplot as plt
import numpy as np
import rospy
from std_msgs.msg import Float64
from numpy.random import randn, random_sample
from scipy.stats import norm
import time
from copy import deepcopy
from simple_filter.msg import LaserSimple, OdometrySimple

class SimpleParticleFilter(object):
    """ This class encompasses the main logic for the particle filter """
    def __init__(self):
        rospy.init_node('simple_particle_filter')
        self.walls = rospy.get_param('~walls')
        self.n_particles = rospy.get_param('~nparticles')
        real_robot = rospy.get_param('~realrobot', False)

        self.world_model = WorldModel(self.walls)
        sensor_model = SensorModel(model_noise_rate=0.05,
                                   odom_noise_rate=0.1,
                                   world_model=self.world_model,
                                   real_robot=real_robot)

        self.fig = plt.figure()
        self.fig.show()
        self.pf = ParticleFilter()

        for i in range(self.n_particles):
            self.pf.add_particle(Particle(position=randn()+1.5,
                                          weight=1/float(self.n_particles),
                                          sensor_model=sensor_model))
        self.last_scan = None
        self.last_odom = None
        self.true_position = None
        rospy.Subscriber('/simple_scan', LaserSimple, self.process_scan)
        rospy.Subscriber('/simple_odom', OdometrySimple, self.process_odom)
        rospy.Subscriber('/true_position', Float64, self.process_true_position)

    def process_scan(self, msg):
        """ Process the simple scans coming from the simulator
            or the Neato bridge """
        self.last_scan = msg

    def process_true_position(self, msg):
        """ This topic is only available when workign with the
            simulator """
        self.true_position = msg.data

    def process_odom(self, msg):
        """ Process odometry messages from the simulator or the
            neato bridge """
        if (self.last_odom != None  and
            msg.south_to_north_position != self.last_odom.south_to_north_position):
            delta = msg.south_to_north_position - self.last_odom.south_to_north_position
            self.pf.predict(delta)
        self.last_odom = msg

    def draw_world_state(self):
        """ Visualize the current state of the world """
        self.fig.clf()
        t = time.time()
        subplot = self.fig.add_subplot(2,1,1)
        for w in self.world_model.walls:
            subplot.plot([w,w],[0,1],'b-')
            subplot.hold(True)
        subplot.set_xlim([min(self.walls)-0.2,max(self.walls)+.2])
        subplot.set_ylim([0,1])
        subplot.scatter([p.position for p in self.pf.particles],
                        [0.5]*len(self.pf.particles),
                        c='r',
                        s=[p.weight**0.5*1000 for p in self.pf.particles])

        subplot.scatter([p.position for p in self.pf.particles],
                        [0.2]*len(self.pf.particles),
                        c='k',
                        s=[10]*len(self.pf.particles))

        if self.true_position != None:
            subplot.scatter([self.true_position], [0.8], c='g', s=[100])

        histogram = self.fig.add_subplot(2,1,2)

        histogram.hist([p.position for p in self.pf.particles],
                       weights=[p.weight for p in self.pf.particles],
                       bins=np.arange(-0.5+min(self.walls),0.5+max(self.walls),.02))

        histogram.set_xlim([-.2+min(self.walls),0.2+max(self.walls)])
        histogram.set_ylim([0,1])
        plt.draw()

    def run(self):
        """ main run loop """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.last_scan != None:
                self.pf.integrate_observation(self.last_scan)
                self.last_scan = None
            self.pf.normalize()
            self.draw_world_state()
            self.pf.resample()
            r.sleep()

class ParticleFilter(object):
    def __init__(self):
        self.particles = []

    def add_particle(self, p):
        self.particles.append(p)

    def normalize(self):
        w_sum = sum([p.weight for p in self.particles])
        [p.normalize_weight(w_sum) for p in self.particles]

    def integrate_observation(self, observation):
        for p in self.particles:
            p.integrate_observation(observation)

    def predict(self, delta):
        for p in self.particles:
            p.predict(delta)

    @staticmethod
    def weighted_values(values, probabilities, size):
        """ Return a random sample of size elements from the set values with the specified probabilities
            values: the values to sample from (numpy.ndarray)
            probabilities: the probability of selecting each element in values (numpy.ndarray)
            size: the number of samples
        """
        bins = np.add.accumulate(probabilities)
        indices = np.digitize(random_sample(size), bins)
        sample = []
        for ind in indices:
            sample.append(deepcopy(values[ind]))
        return sample

    def resample(self):
        """ resample the particles based on the associated
            probabilities """
        self.particles = ParticleFilter.weighted_values(self.particles,
                                                        [p.weight for p in self.particles],
                                                        len(self.particles))
        for p in self.particles:
            p.weight = 1./len(self.particles)

class SensorModel(object):
    def __init__(self, model_noise_rate, odom_noise_rate, world_model, real_robot):
        self.model_noise_rate = model_noise_rate
        self.odom_noise_rate = odom_noise_rate
        self.world_model = world_model
        self.real_robot = real_robot

    def get_likelihood(self, observation, position, direction):
        """ Returns the probability of a given distance reading
            (observation) in the specified direction if the robot
            was at the specified position """
        if self.real_robot and observation == 0.0:
            return 1.0

        closest = self.world_model.get_closest_wall(position, direction)
        if closest == None and observation == 0.0:
            # probability of a false positive is 0
            return 1.0
        elif closest != None and observation == 0.0:
            # probability of missing an obstacle is 0
            return 0.0
        elif closest == None and observation != 0.0:
            #probability of  false positive is 0
            return 0.0
        return norm(0, self.model_noise_rate).pdf(abs(position - closest) - observation)

    def sample_prediction(self, predicted_position):
        """ Sample a potential next state based on a predicted position
            based off of the Odometry """
        return predicted_position + randn()*self.odom_noise_rate

class WorldModel(object):
    """ A model of our simple 1d world of walls with a robot
        with a forward and backward facing range sensor. """

    def __init__(self, walls=None):
        if walls == None:
            self.walls = []
        else:
            self.walls = walls

    def add_wall(self, wall_position):
        """ Add a wall to the world """
        self.walls.append(wall_position)

    def get_closest_wall(self, position, direction):
        """ Helper function to get the closest obstacle to the
            North or the South of a specified position """
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

class Particle(object):
    """ Represents a particle """
    def __init__(self, position, weight, sensor_model):
        self.position = position
        self.weight = weight
        self.sensor_model = sensor_model

    def integrate_observation(self, observation):
        """ integrate an observation """
        self.weight *= self.sensor_model.get_likelihood(observation.north_laser, self.position, 1)
        self.weight *= self.sensor_model.get_likelihood(observation.south_laser, self.position, -1)

    def predict(self, delta):
        """ predict the next position based on the delta measured using
            the odometry """
        self.position  = self.sensor_model.sample_prediction(self.position+delta)

    def normalize_weight(self, Z):
        """ adjust the particle weight using the specified
            normalization factor (Z) """
        self.weight /= Z

if __name__ == '__main__':
    node = SimpleParticleFilter()
    node.run()