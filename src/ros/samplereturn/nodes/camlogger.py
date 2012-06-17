#!/usr/bin/env python
import roslib; roslib.load_manifest('samplereturn')
from photo.srv import *
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image,CameraInfo
import rospy
import numpy as np

def capture_image():
  rospy.logdebug("waiting for photo service")
  rospy.wait_for_service('photo_node/capture')
  try:
    rospy.logdebug("requesting photo")
    capture = rospy.ServiceProxy('photo_node/capture', Capture, persistent=True)
    img = capture()
    rospy.logdebug("received photo")
    return img.image
  except rospy.ServiceException, e:
    print "Service Call Failed: %s"%e

def camlogger():
  topic = 'camimg'
  pub = rospy.Publisher(topic,Image)
  #info_pub = rospy.Publisher(info_topic,CameraInfo)
  rospy.init_node('camlogger',log_level=rospy.DEBUG)
  r = rospy.Rate(1)
  while not rospy.is_shutdown():
    img = capture_image()
    rospy.logdebug("img (%d, %d)", img.width, img.height)
    pub.publish(img)
    r.sleep()

if __name__=="__main__":
  try:
    camlogger()
  except rospy.ROSInterruptException: pass
