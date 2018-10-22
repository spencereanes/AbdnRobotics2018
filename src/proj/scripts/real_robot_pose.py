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


class real_robot_pose:
  def __init__(self):
    rospy.init_node('real_robot_pose')
    self.sub=rospy.Subscriber('/base_pose_ground_truth', Odometry, self.handle_pose)
    self.listener=tf.TransformListener()
    self.br=tf.TransformBroadcaster()
    self.r=rospy.Rate(10)
    self.first=True
    self.m=marker.Markers()
    self.position=Pose()
    
    while not rospy.is_shutdown():
      self.broadcast()
      self.real_pos_marker()
      #print self.position.position
      self.r.sleep()

  def handle_pose(self, data):
    self.position=data.pose.pose
    
  def broadcast(self):
    p=self.position.position
    q=self.position.orientation
    try:
      self.br.sendTransform((p.x,p.y,p.z), (q.x,q.y,q.z,q.w), rospy.Time.now(), "/real_robot_pose", "/map")
    except:
      print 'ERROR: Broadcaster'

    if(self.first):
      rospy.sleep(.5)
      self.first=False

  def real_pos_marker(self):
    p=self.position.position
    self.m.add(p.x+6,p.y+4.8,1,0,0,'real_robot_pose')
    self.m.draw()
    self.r.sleep()

bot=real_robot_pose()
