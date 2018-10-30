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
from geometry_msgs.msg import PointStamped
#import twist

class contoller:
  
  self.sub=rospy.Subscriber('/real_robot_pose',Odometry,self.handle_pose)
  self.pub=rospy.Publisher('/cmd_vel',Twist)
  
  
