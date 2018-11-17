#!/usr/bin/env python
import tf
import rospy
import roslib
import math
import time

import marker

from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

class controller:
  def __init__(self,path,tolerance_x=.5,tolerance_t=1):
    self.dt=.05
    self.r=rospy.Rate(1/self.dt)
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
    self.tolerance_x=tolerance_x
    self.tolerance_t=tolerance_t
    self.orientation=0

    self.follow_path()
  
  def handle_amcl_pose(self,data):
    self.position=data.pose.pose.position
    self.orientation=tf.transformations.euler_from_quaternion([0.,0.,data.pose.pose.orientation.z,data.pose.pose.orientation.w])[2]
    #print self.orientation

  def follow_path(self):
    ctr=0
    while len(self.path) > 0 and not rospy.is_shutdown():
      ctr+=1
      print(ctr)
      for i in range(10):
        curr_goal = self.path.pop()
      goal_theta = self.update_goal_theta(curr_goal)
      #self.draw_path()
      #self.m.draw()
      self.driver(curr_goal,goal_theta)

  #def pid_control(self,dt,

  def update_goal_theta(self,curr_goal):
    return math.atan2(curr_goal[1]-self.position.y,curr_goal[0]-self.position.x)

  #currently using only a p controller with a couple tweaks
  def driver(self,goal,goal_theta):

    #define control parameters
    kpt=1
    kit=0
    kdt=0
    kpx=1
    kix=0
    kdx=0

    #defining initial values
    int_t=0
    int_x=0
    e0_t=0
    e0_x=0
    p=[self.position.x,self.position.y]
    #error in theta
    error_t = goal_theta - self.orientation
    #error in position
    error_x = self.dist(p,goal)
    print "error_x: ", error_x
    print "error_t: ", error_t
    print

    while ((error_x>self.tolerance_x) or (abs(error_t) > self.tolerance_t)):
      print "error_x: ", error_x
      print "error_t: ", error_t
      print

      p=[self.position.x,self.position.y]
      #update errors and position
      error_t = goal_theta - self.orientation
      error_x = self.dist(p,goal)

      #angular PID controller
      int_t = int_t + (error_t * self.dt)
      deriv_t = (error_t - e0_t)/self.dt
      self.movement.angular.z = kpt*error_t + kit*int_t + kdt*deriv_t

      #linear PID controller
      int_x = int_x + (error_x * self.dt)
      deriv_x = (error_x - e0_x)/self.dt
      self.movement.linear.x= kpx*error_x + kix*int_x + kdx*deriv_x
      
      
      if abs(error_t) > math.pi/3:
        self.movement.linear.x = 0
      self.movement.linear.x=min(.5,self.movement.linear.x)
      if self.movement.angular.z > 0:
        self.movement.angular.z=min(math.pi/4,self.movement.angular.z)
      else:
        self.movement.angular.z=max(-math.pi/4,self.movement.angular.z)
      
      self.pub.publish(self.movement)
      #print self.movement
      #print
      e0_t=error_t
      e0_x=error_x

      goal_theta=self.update_goal_theta(goal)
      #potential issue: if not enough movement, amcl won't publish update
      #meaning the two errors would match
      self.r.sleep()

    #goal is reached, stop movement, sleep and move on
    self.movement.linear.x=0
    self.movement.angular.z=0
    self.pub.publish(self.movement)
    self.r.sleep()

    print
    print "Path: ",self.path[-2:-1]
    print "Position: ", [self.position.x,self.position.y]

    return


  def dist_p(self,v1,v2):
    return math.sqrt(v1**2 + v2**2)

  def dist(self,p1,p2):
    return math.sqrt(abs(p1[0]-p2[0])**2+abs(p1[1]-p2[1]))

  def draw_path(self):
    self.m=marker.Markers("/path")
    for subpath in self.p:
      a=range(len(subpath))
      for i in a[0::10]:
        self.m.add(subpath[i][0],subpath[i][1],1,0,0,'map')

