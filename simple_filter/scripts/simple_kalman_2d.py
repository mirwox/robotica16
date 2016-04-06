#!/usr/bin/env python

"""
    This script implements a Kalman filter for the system:

    x_0 ~ N(0, sigma_sq)
    x_t = x_{t-1} + w_t, w_t ~ N(0, sigma_m_sq)
    z_t = x_t + v_t, v_t ~ N(0, sigma_z_sq)
"""

import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
import rospy
import numpy as np
from numpy.random import randn
from math import e, sqrt, pi
from dynamic_reconfigure.server import Server
from simple_filter.cfg import SimpleKalmanConfig
from numpy.random import multivariate_normal

class SimpleWorld(object):
    """ A simple system with dynamics:
        x_0 ~ N(0, sigma_sq)
        x_t = [1 0; 0 .1]*x_{t-1} + w_t, w_t ~ N(0, [0 0; 0 sigma_m_sq])
        z_t = x_t + v_t, v_t ~ N(0, sigma_z_sq)
    """

    def __init__(self, mu_0, sigma_0, sigma_m_sq, sigma_z_sq):
        """ the initial state is sampled from N(mu_0, sigma_0).
            the movement noise is sigma_m_sq and the measurement noise is sigma_z_sq
        """
        self.x_true = multivariate_normal(mu_0, sigma_0)
        self.sigma_m_sq = [[0, 0], [0, sigma_m_sq]]
        self.sigma_z_sq = [[sigma_z_sq]]
        self.A = np.asarray([[1, .1],[0, 1]])
        self.H = np.asarray([[0, 1]])

    def get_z_t(self):
        """ Sample an observation centered at x_true plus Gaussian noise
            with variance sigma_sq_z and mean 0 """
        return multivariate_normal(self.H.dot(self.x_true), self.sigma_z_sq)

    def get_x_t(self):
        """ Sample next system state as the current system state plus Gaussian
            noise with variance sigma_sq_m and mean 0 """
        self.x_true = self.A.dot(self.x_true) + multivariate_normal([0,0], self.sigma_m_sq)
        return self.x_true

class SimpleKalmanFilter(object):
    """ A Kalman filter node that estimates a single state x_t using noisy position measurements """

    def __init__(self):
        """ Sets up the world model and loads initial parameters """
        rospy.init_node('simple_kalman')
        plt.ion()

        # initial beliefs
        self.mu = np.asarray([0, 3])
        self.sigma_sq = np.asarray([[1, 0],[0, 4]])

        # motor noise
        sigma_m_sq = rospy.get_param('~sigma_m_sq', 0.01)
        # observation noise
        sigma_z_sq = rospy.get_param('~sigma_z_sq', .1)

        # time to pause between plots
        self.pause_time = rospy.get_param('~pause_time', 0.5)

        self.graphs = None
        self.world = SimpleWorld(self.mu, self.sigma_sq, sigma_m_sq, sigma_z_sq)

        srv = Server(SimpleKalmanConfig, self.config_callback)

    def config_callback(self, config, level):
        """ Get the pause_time, movement noise, and measurement noise """
        print "GOT RECONFIG REQUEST"
        self.pause_time = config['pause_time']
        self.world.sigma_m_sq = [[0, 0], [0, config['sigma_m_sq']]]
        self.world.sigma_z_sq = [[config['sigma_z_sq']]]
        return config

    def run(self):
        while not rospy.is_shutdown():
            # Graph new observation from the system
            z_t = self.world.get_z_t()
            self.plot_pdf(z_t)

            # Do Kalman updates
            self.mu = self.world.A.dot(self.mu)
            self.sigma_sq = self.world.A.dot(self.sigma_sq).dot(self.world.A.T) + self.world.sigma_m_sq

            measurement_residual = z_t - self.world.H.dot(self.mu)
            residual_covariance = self.world.H.dot(self.sigma_sq).dot(self.world.H.T) + self.world.sigma_z_sq
            K_t = self.sigma_sq.dot(self.world.H.T).dot(np.linalg.inv(residual_covariance))
            self.mu = self.mu + K_t.dot(measurement_residual)

            self.sigma_sq = (np.eye(len(self.mu))-K_t.dot(self.world.H)).dot(self.sigma_sq)
            plt.pause(self.pause_time)
            self.plot_pdf(z_t)
            # sample next state
            self.world.get_x_t()
            plt.pause(self.pause_time)

    def plot_pdf(self, z):
        """ Plot the Gaussian PDF with the specified mean (mu) and variance (sigma_sq)
            x_true is the true system state which will be plotted in blue
            z is the current observation which will be plotted in red """
        try:
            delta = .01
            x = np.arange(self.mu[0]-5, self.mu[0]+5, delta)
            y = np.arange(self.mu[1]-5, self.mu[1]+5, delta)
            X, Y = np.meshgrid(x, y)
            Z = mlab.bivariate_normal(X,Y,self.sigma_sq[0,0], self.sigma_sq[1,1], self.mu[0], self.mu[1], self.sigma_sq[0,1])
            plt.clf()
            CS = plt.contour(X, Y, Z)
            plt.clabel(CS, fontsize=9)
            print self.sigma_sq
            plt.xlabel('position')
            plt.ylabel('velocity')
            plt.title('Probabity Density')
            plt.show()
        except:
            print "couldn't make plot"

if __name__ == '__main__':
    node = SimpleKalmanFilter()
    node.run()