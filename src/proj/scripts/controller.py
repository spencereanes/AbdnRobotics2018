#!/usr/bin/env python
import tf
import rospy
import roslib
import math
import numpy as np
import time

import marker
from PriorityQueue import *

from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
#import twist

class controller:
  def __init__(self,path,tolerance=.024):
    self.r=rospy.Rate(20)
    self.path=path
    self.p=list(self.path)

    rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
    #amcl is ready
    rospy.wait_for_message("/odom",Odometry)
    #odom running and ready

    self.sub=rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,self.handle_amcl_pose)
    self.pub=rospy.Publisher('/cmd_vel',Twist,queue_size=15)

    temp = self.path.pop()
    self.position=Point(temp[0],temp[1],0)
    self.movement=Twist()
    self.tolerance=tolerance
    self.goal_angle=0
    self.orientation=0

    self.follow_path()
  
  def handle_amcl_pose(self,data):
    self.position=data.pose.pose.position
    self.orientation=tf.transformations.euler_from_quaternion([0.,0.,data.pose.pose.orientation.z,data.pose.pose.orientation.w])[2]
    #print self.orientation
    
  def draw_path(self):
    self.m=marker.Markers("/path")
    for subpath in self.p:
      a=range(len(subpath))
      for i in a[0::10]:
        self.m.add(subpath[i][0],subpath[i][1],1,0,0,'map')

  def dist_p(self,v1,v2):
    return math.sqrt(v1**2 + v2**2)

  def dist(self,p1,p2):
    return math.sqrt(abs(p1[0]-p2[0])**2+abs(p1[1]-p2[1]))

  def follow_path(self):
    ctr=0
    while len(self.path) > 0 and not rospy.is_shutdown():
      ctr+=1
      print(ctr)
      curr_goal = self.path.pop()
      self.update_goal_theta(curr_goal)
      #self.draw_path()
      #self.m.draw()
      self.driver(curr_goal)
      self.r.sleep()

  def update_goal_theta(self,curr_goal):
    self.goal_theta = math.atan2(curr_goal[1]-self.position.y,curr_goal[0]-self.position.x)

  def driver(self,goal):
    print
    print "Path: ",self.path[-2:-1]
    print "Position: ", [self.position.x,self.position.y]

    p=[self.position.x,self.position.y]
    self.movement.angular.z = self.goal_theta - self.orientation
 
    if self.movement.angular.z > .6:
      self.movement.linear.x=0
      self.pub.publish(self.movement)
      self.update_goal_theta(goal)
      self.r.sleep()
      self.driver(goal)

    if self.dist(p,goal)<self.tolerance:
      self.movement.linear.x=0
      while not abs(self.orientation - self.goal_theta) > .1:
        self.movement.angular.z = self.goal_theta - self.orientation
        self.pub.publish(self.movement)
        self.r.sleep()

    else:
      print 'Moving'
      dx = goal[0]-p[0]
      dy = goal[1]-p[1]
      self.movement.linear.x=self.dist(p,goal)/2
      
      #print("movement: ", self.movement)
      #set linear and angular speeds proportional to distance frome goal
    self.pub.publish(self.movement)
    return
      #try:
      
    """
      except Exception as e:
        print "exception in executing driver"
        print(e)
        return
    """

