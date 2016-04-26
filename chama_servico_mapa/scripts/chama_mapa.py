#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap

def obter_mapa():
  rospy.wait_for_service('static_map')
  try:
     get_map = rospy.ServiceProxy('static_map', GetMap)
     mapa = get_map().map
     return mapa
  except rospy.ServiceException, e:
     print "Service call failed: %s"%e


if __name__=="__main__":
	#rospy.
	pass