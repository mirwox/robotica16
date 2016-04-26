#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy import optimize
from matplotlib import pyplot as plt, cm, colors
from math import pi, atan2
import math
import dynamic_reconfigure.client
from tf.transformations import euler_from_quaternion

def calc_R(x,y, xc, yc):
    """ calculate the distance of each 2D points from the center (xc, yc) """
    return np.sqrt((x-xc)**2 + (y-yc)**2)

def f(c, x, y):
    """ calculate the algebraic distance between the data points and the mean circle centered at c=(xc, yc) """
    Ri = calc_R(x, y, *c)
    return Ri - Ri.mean()

def leastsq_circle(x,y):
    # coordinates of the barycenter
    x_m = np.mean(x)
    y_m = np.mean(y)
    center_estimate = x_m, y_m
    center, ier = optimize.leastsq(f, center_estimate, args=(x,y))
    xc, yc = center
    Ri       = calc_R(x, y, *center)
    R        = Ri.mean()
    residu   = np.sum((Ri - R)**2)
    return xc, yc, R, residu

def plot_data_circle(x,y, xc, yc, R):
    #f = plt.figure( facecolor='white')  #figsize=(7, 5.4), dpi=72,
    plt.clf()
    plt.axis('equal')

    theta_fit = np.linspace(-pi, pi, 180)

    x_fit = xc + R*np.cos(theta_fit)
    y_fit = yc + R*np.sin(theta_fit)
    plt.plot(x_fit, y_fit, 'b-' , label="fitted circle", lw=0.5)
    plt.plot([xc], [yc], 'bD', mec='y', mew=1)
    plt.xlabel('x')
    plt.ylabel('y')   
    # plot data
    plt.plot(x, y, 'r-.', label='data', mew=1)

def angle_normalize(z):
    """ convenience function to map an angle to the range [-pi,pi] """
    return math.atan2(math.sin(z), math.cos(z))

def angle_diff(a, b):
    """ Calculates the difference between angle a and angle b (both should be in radians)
        the difference is always based on the closest rotation from angle a to angle b
        examples:
            angle_diff(.1,.2) -> -.1
            angle_diff(.1, 2*math.pi - .1) -> .2
            angle_diff(.1, .2+2*math.pi) -> -.1
    """
    a = angle_normalize(a)
    b = angle_normalize(b)
    d1 = a-b
    d2 = 2*math.pi - math.fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    if math.fabs(d1) < math.fabs(d2):
        return d1
    else:
        return d2

class CalibrateStarPose(object):
    def __init__(self):
        rospy.init_node('calibrate_star_pose')
        rospy.Subscriber('STAR_pose', PoseStamped, self.process_pose)
        self.pts = []

    def process_pose(self, msg):
        euler_angles = euler_from_quaternion((msg.pose.orientation.x,
                                              msg.pose.orientation.y,
                                              msg.pose.orientation.z,
                                              msg.pose.orientation.w))
        pt = (msg.pose.position.x, msg.pose.position.y, euler_angles[2])
        self.pts.append(pt)

    def get_phase_offset(self,xc,yc,R,xs,ys,yaws):
        phases = np.zeros(len(self.pts),)
        for i in range(len(xs)):
            angle_to_center = atan2(yc - ys[i], xc - xs[i])
            phases[i] = angle_diff(angle_to_center, yaws[i]+pi)
        return np.median(phases)

    def config_callback(self, config):
        pass

    def run(self):
        client = dynamic_reconfigure.client.Client("star_center_positioning_node", timeout=30, config_callback=self.config_callback)
        client.update_configuration({"pose_correction":0.0, "phase_offset":0.0})
        r = rospy.Rate(5)
        plt.figure()
        while not rospy.is_shutdown():
            xs = [pt[0] for pt in self.pts]
            ys = [pt[1] for pt in self.pts]
            yaws = [pt[2] for pt in self.pts]
            min_dim = min(len(xs), len(ys), len(yaws))
            xs = xs[:min_dim]
            ys = ys[:min_dim]
            yaws = yaws[:min_dim]
            if len(xs) > 5:
                xc,yc,R,residu = leastsq_circle(np.asarray(xs),np.asarray(ys))
                phase_offset = self.get_phase_offset(xc,yc,R,xs,ys,yaws)

                print "rosrun my_pf star_center_position_revised.py _pose_correction:="+str(R) + " _phase_offset:="+str(phase_offset)
                plot_data_circle(xs,ys,xc,yc,R)

            plt.pause(.05)
            r.sleep()
        client.update_configuration({"pose_correction":R, "phase_offset":phase_offset})


if __name__ == '__main__':
    node = CalibrateStarPose()
    node.run()