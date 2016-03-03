#!/usr/bin/env python

import rospy
from nav_msgs import Odometry
from geometry_msgs import Twist
from math import fabs


velocidade_objetivo = None;
pub = rospy.Publisher('/cmd_vel', Twist)

def notificacao(data):
    """
        Codigo de notificacao executado sempre que chega uma leitura da odometria

        Esta leitura chega na variavel data e e'  um objeto do tipo odometria
    """

    pass




controle():
    rospy.Subscriber('/odom', Odometry, notificacao).
    # Initial movement.
    pub.publish(velocidade_objetivo)
    rospy.spin() # Faz um loop infinito para o ROS nao retornar


if __name__ == '__main__':
    try:
        controle()
    except rospy.ROSInterruptException:
        pass
