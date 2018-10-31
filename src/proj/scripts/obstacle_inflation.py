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
    self.marr=np.array(self.data).reshape(self.height,self.width)
    self.res=self.map.info.resolution
    print 'here'

    inflated_data=(self.obstacle_inflation1()).astype(int)
    inflated_data=np.insert(inflated_data,0,[self.height,self.width,self.res])
    np.savetxt("inflated_data.csv",inflated_data,delimiter=",")
    rospy.loginfo("done")

  #this implementation is very slow
  def obstacle_inflation(self,dist=10):
    copy=self.marr.copy()
    for x in range(self.width):
      #print x
      for y in range(self.height):
        if self.marr[x][y] == 100:
          for x1 in range(min(0,x-dist),min(self.width,x+dist)):
            for y1 in range(min(0,y-dist),min(self.width,y+dist)):
              copy[x1,y1]=100

    return copy

  #this one is slightly faster
  def obstacle_inflation1(self,dist=8):
    copy=self.marr.copy()
    for x in range(self.height):
      print x
      for y in range(self.width):
        if self.marr[x,y]==100:
          continue
        if sum(self.marr[min(0,x-dist):min(self.width,x+dist), min(0,y-dist):min(self.width,y+dist)].sum(axis=0)) >=100:
          copy[x,y]=100

    return copy

go=obstacle_inflation()
