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

    inflated_data=(self.obstacle_inflation())
    inflated_data=np.insert(inflated_data,0,[self.width,self.height,self.res])
    np.savetxt("inflated_data.csv",inflated_data,fmt='%.3f',delimiter=",")
    rospy.loginfo("done")

  #this implementation is very slow
  def obstacle_inflation(self,dist=1):
    copy=list(self.data)
    for x in range(self.width):
      print x
      for y in range(self.height):
        #print x*1000+y
        if self.data[x*800+y] == 100:
          for x1 in range(max(0,x-dist),min(self.height,x+dist)):
            for y1 in range(max(0,y-dist),min(self.width,y+dist)):
              copy[y*1000+x]=100
              
    return copy

  #this one is slightly faster
  def obstacle_inflation1(self,dist=2):
    copy=list(self.data)
    for x in range(self.width):
      print x
      for y in range(self.height):
        if self.data[y*1000+i]==100:
          continue
        if sum(self.marr[min(0,x-dist):min(self.width,x+dist), min(0,y-dist):min(self.width,y+dist)].sum(axis=0)) >=100:
          #print "(x,y): ",x,y
          copy[x,y]=100

    return copy

go=obstacle_inflation()
