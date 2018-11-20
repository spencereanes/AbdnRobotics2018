#!/usr/bin/env python
import tf
import rospy
import roslib
import math

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError

"""
http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
https://github.com/jstnhuang/ros-rviz/wiki/User-guide

This code works in conjunction with task5.launch
It is used to capture stage published camera information, make some adjustments,
and the topics it publishes on are used by task5.launch to create a depthcloud.
"""

class camera:
  def __init__(self):
    rospy.init_node('camera')
    self.image_pub=rospy.Publisher('/camera/image_raw',Image,queue_size=15)
    self.depth_pub=rospy.Publisher('/camera/depth_raw',Image,queue_size=15)
    self.camera_info_pub=rospy.Publisher('/camera/camera_info',CameraInfo,queue_size=15)

    self.bridge=CvBridge()

    rospy.wait_for_message('/depth',Image)
    self.depth_sub=rospy.Subscriber('/depth',Image,self.handle_depth)
    rospy.wait_for_message('/camera_info',CameraInfo)
    
  
    self.image_sub=rospy.Subscriber('/image',Image,self.handle_image)
    self.cam_info_sub=rospy.Subscriber('/camera_info',CameraInfo,self.handle_info)

    rospy.spin()

  """
  Was having issues with depth_image_proc/convert_metric so I tried a work around
  using cv_bridge trying to circumvent the problem... Then I realized that I
  want the depth data as 32FC1, so I don't have to use the convert_metric function at all.

  #transform coordinates from camera frame to global frame
  """
  def handle_depth(self,depth_data):
    """
    try:
      cv_image = self.bridge.imgmsg_to_cv2(depth_data, '32FC1')
    except CvBridgeError as e:
      print(e)
    
    try:
      converted_depth = self.bridge.cv2_to_imgmsg(cv_image, "passthrough")
    except CvBridgeError as e:
      print(e)
    print converted_depth.encoding
    """
    print "depth data encoding: ", depth_data.encoding
    self.depth_pub.publish(depth_data)

  def handle_image(self,image_data):
    print "image data encoding: ", image_data.encoding
    self.image_pub.publish(image_data)

  #this is republishing /camera_info to /camera/camera_info for image_proc in task5.launch
  def handle_info(self,info_data):
    self.camera_info_pub.publish(info_data)



cam=camera()
