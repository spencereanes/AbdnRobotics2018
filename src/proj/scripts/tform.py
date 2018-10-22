#!/usr/bin/env python
import tf
import rospy
import roslib

import math
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class tform:
  def __init__(self):
    rospy.init_node('tform')
    self.listener = tf.TransformListener()
    self.pub = rospy.Publisher('pos_in_map',Odometry,queue_size=1)
    self.sub = rospy.Subscriber('real_robot_pose',Odometry,self.processor) 
    #self.width=.5
    #self.min_range=.35
