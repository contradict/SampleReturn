#!/usr/bin/env python
import roslib
import rospy
import rospkg
import numpy as np
import cv2

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

def image_publisher():
    rospy.init_node('image_publisher')
    pub = rospy.Publisher('static_image',Image,queue_size=3)
    bridge = CvBridge()
    img = cv2.imread("/home/zlizer/src/SampleReturn/ros_workspace/park_dslr/pre_cached/frame0015.jpg")
    img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        print "send"
        pub.publish(bridge.cv2_to_imgmsg(img,'rgb8'))
        rate.sleep()

if __name__=="__main__":
  try:
    image_publisher()
  except rospy.ROSInterruptException: pass
