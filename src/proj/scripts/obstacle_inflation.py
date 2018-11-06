#!/usr/bin/env python
import rospy
import roslib
import math
import numpy as np

from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped

class obstacle_inflation:
  def __init__(self):
    rospy.init_node('map_data')
    self.map=rospy.wait_for_message('/map',OccupancyGrid,timeout=None)
    rospy.loginfo('\nMap information collected')

    self.data=self.map.data
    self.width=self.map.info.width
    self.height=self.map.info.height
    self.res=self.map.info.resolution
    print 'here'

    inflated_data=self.obstacle_inflation()
    np.savetxt("/home/viki/catkin_ws/src/proj/scripts/inflated_data.csv",inflated_data,fmt='%.3f',delimiter=",")
    rospy.loginfo("done")

  def obstacle_inflation(self,dist=10):
    copy=list(self.data)
    for x in range(self.width):
      print x
      for y in range(self.height):
        if self.data[y*1000+x] == 100:
          for x1 in range(max(0,x-dist),min(self.width,x+dist)):  
            for y1 in range(max(0,y-dist),min(self.height,y+dist)):
              #print "(x1,y1): " , x1, y1
              copy[y1*1000+x1]=100
              
    return copy

go=obstacle_inflation()
