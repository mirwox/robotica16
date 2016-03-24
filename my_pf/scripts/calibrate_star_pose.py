#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy import optimize
from matplotlib import pyplot as plt, cm, colors
from math import pi

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


class CalibrateStarPose(object):
    def __init__(self):
        rospy.init_node('calibrate_star_pose')
        rospy.Subscriber('/STAR_pose', PoseStamped, self.process_pose)
        self.pts = []

    def process_pose(self, msg):
        pt = (msg.pose.position.x, msg.pose.position.y)
        self.pts.append(pt)

    def run(self):
        r = rospy.Rate(5)
        plt.figure()
        while not rospy.is_shutdown():
            xs = [pt[0] for pt in self.pts]
            ys = [pt[1] for pt in self.pts]
            min_dim = min(len(xs), len(ys))
            xs = xs[:min_dim]
            ys = ys[:min_dim]
            if len(xs) > 5:
                xc,yc,R,residu = leastsq_circle(np.asarray(xs),np.asarray(ys))
                print "rosrun my_pf star_center_position.py _pose_correction:="+str(R)
                plot_data_circle(xs,ys,xc,yc,R)

            plt.pause(.05)
            r.sleep()



if __name__ == '__main__':
    node = CalibrateStarPose()
    node.run()