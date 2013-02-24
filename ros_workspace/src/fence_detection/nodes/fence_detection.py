#!/usr/bin/python
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from nav_msgs.msg import GridCells
from cv_bridge import CvBridge

import rospy
import numpy as np
import cv2
from copy import deepcopy

class fence_detection(object):
  def __init__(self):
    rospy.init_node('fence_detection',anonymous=True, log_level=rospy.INFO)
    self.img_sub = rospy.Subscriber('/navigation/left/image_rect_color',Image,self.detect_fence)
    self.disp_sub = rospy.Subscriber('/navigation/disparity',DisparityImage,self.handle_disp)

    self.img_pub = rospy.Publisher('fence_img',Image)
    self.disp_pub = rospy.Publisher('fence_disp',DisparityImage)

    self.bridge = CvBridge()

    self.min_h = rospy.get_param('~min_h',5)
    self.max_h = rospy.get_param('~max_h',16)
    self.min_s = rospy.get_param('~min_s',150)
    self.max_s = rospy.get_param('~max_s',180)

    self.disp_img = None

  def detect_fence(self,Image):
    img = np.asarray(self.bridge.imgmsg_to_cv(Image,'bgr8'))
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    thresh = cv2.inRange(hsv[:,:,:2],np.array([self.min_h,self.min_s]),np.array([self.max_h,self.max_s]))/255

    masked_img = np.multiply(img,thresh[...,None])

    if self.disp_img == None:
      return

    masked_disp_img = np.multiply(self.disp_img,thresh).astype(np.float32)
    self.disp_out = DisparityImage()
    self.disp_out.min_disparity = self.min_disparity
    self.disp_out.max_disparity = self.max_disparity
    self.disp_out.image = self.bridge.cv_to_imgmsg(cv2.cv.fromarray(masked_disp_img))
    # For some reason, cv_bridge is setting the step to 640, not 4*640
    # fixing this manually
    self.disp_out.image.step = 2560
    self.disp_out.header = self.header

    img_out = self.bridge.cv_to_imgmsg(cv2.cv.fromarray(masked_img),'bgr8')
    img_out.header = Image.header

    self.img_pub.publish(img_out)
    self.disp_pub.publish(self.disp_out)

  def handle_disp(self,DisparityImage):
    self.disp_img = np.asarray(self.bridge.imgmsg_to_cv(DisparityImage.image))
    self.min_disparity = DisparityImage.min_disparity
    self.max_disparity = DisparityImage.max_disparity
    self.header = DisparityImage.header

  #def extract_points(self):

if __name__=="__main__":
  try:
    fence_detection()
    rospy.spin()
  except rospy.ROSInterruptException: pass
