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

class contoller:
  def __init__(self,path,tolerance=.036):
    rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
    #amcl is ready
    rospy.wait_for_message("/odom",Odometry)
    #odom running and ready
    self.sub=rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,self.handle_amcl_pose)
    self.pub=rospy.Publisher('/cmd_vel',Twist,queue_size=15)
    self.position=path.pop()
    self.movement=Twist()
    self.tolerance=tolerance
  
  def handle_amcl_pose(self,data):
    self.position=data.pose.pose.position
    self.orientation=data.pose.pose.orientation
    

  def driver(self,pos):
    if abs(self.position-pos)<self.tolerance:
      self.movement.linear.x=0
      self.movement.angular.z=0
    else:
      #set linear and angular speeds proportional to distance frome goal

    self.pub.publish(self.movement)


