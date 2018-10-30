#!/usr/bin/env python
import tf
import rospy
import roslib
import math
import marker

from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseWithCovarianceStamped


class real_robot_pose:
  def __init__(self):

    rospy.init_node('real_robot_pose')

    self.real_pose=rospy.Subscriber('/base_pose_ground_truth', Odometry, self.handle_real_pose)
    self.amcl_pose=rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,self.handle_amcl_pose)

    self.listener=tf.TransformListener()
    self.br=tf.TransformBroadcaster()
    self.r=rospy.Rate(10)

    self.first=True

    self.m=marker.Markers()

    self.real_pose=Odometry()
    self.amcl_pose=PoseWithCovarianceStamped()
    
    while not rospy.is_shutdown():
      self.broadcast()
      self.real_pose_marker()
      self.amcl_pose_marker()
      self.m.draw()
      self.r.sleep()
      #self.m.clear()

  def handle_real_pose(self, data):
    self.real_pose=data

  def handle_amcl_pose(self,data):
    self.amcl_pose=data
    
  def broadcast(self):
    p=self.real_pose.pose.pose.position
    q=self.real_pose.pose.pose.orientation
    try:
      self.br.sendTransform((p.x,p.y,p.z), (q.x,q.y,q.z,q.w), rospy.Time.now(), "/real_robot_pose", "/map")
    except:
      print 'ERROR: Broadcaster'

    #this makes sure that the first pass we are broadcasting before we look for the frame
    if(self.first):
      rospy.sleep(.5)
      self.first=False

  def real_pose_marker(self):
    p=self.real_pose.pose.pose.position
    self.m.add(p.x,p.y,1,0,0,'map')

  def amcl_pose_marker(self):
    p=self.amcl_pose.pose.pose.position
    self.m.add(p.x,p.y,0,0,1,'map')

bot=real_robot_pose()
