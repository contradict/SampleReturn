#!/usr/bin/env python
import roslib; roslib.load_manifest('samplereturn')
from photo.srv import *
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image
import rospy
import numpy as np

def capture_image():
  rospy.wait_for_service('photo/capture')
  try:
    capture = rospy.ServiceProxy('photo/capture', Capture, persistent=True)
    img = capture()
    return img.image
  except rospy.ServiceException, e:
    print "Service Call Failed: %s"%e

def camlogger():
  topic = 'camimg'
  pub = rospy.Publisher(topic,Image)
  rospy.init_node('camlogger')
  while not rospy.is_shutdown():
    img = capture_image()
    pub.publish(img)
    rospy.sleep(3.0)

if __name__=="__main__":
  try:
    camlogger()
  except rospy.ROSInterruptException: pass
