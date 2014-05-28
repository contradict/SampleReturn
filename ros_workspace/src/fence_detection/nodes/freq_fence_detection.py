#!/usr/bin/python
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge

import rospy
import numpy as np
import cv2
from copy import deepcopy
import scipy.io
import rospkg

class freq_fence_detection(object):
  def __init__(self):
    rospy.init_node('freq_fence_detection',anonymous=True, log_level=rospy.INFO)
    self.img_sub = rospy.Subscriber('input_image',Image,self.detect_fence)
    self.disp_sub = rospy.Subscriber('input_disparity',DisparityImage,self.handle_disp)

    self.img_pub = rospy.Publisher('fence_image',Image)
    self.disp_pub = rospy.Publisher('fence_disparity',DisparityImage)

    self.bridge = CvBridge()

    self.disp_img = None

    #self.mat = scipy.io.loadmat('/home/zlizer/Downloads/ColorNaming/w2c.mat')
    rospack = rospkg.RosPack()
    self.mat = scipy.io.loadmat(rospack.get_path('fence_detection')+'/w2c.mat')

  def detect_fence(self,Image):
    img = np.asarray(self.bridge.imgmsg_to_cv(Image,'rgb8'))
    lab = cv2.cvtColor(img, cv2.COLOR_RGB2LAB)
    lab = lab[...,1]
    mask = np.zeros_like(lab)
    masked_img = img.copy()

    #Patch check for frequency
    #Color pass on low-freq patches
    for i in range(0,12):
      for j in range(0,16):
        patch = lab[i*40:(i+1)*40,j*40:(j+1)*40]
        img_patch = img[i*40:(i+1)*40,j*40:(j+1)*40]
        #patch_dct = cv2.dct(patch.astype(np.float32))**2
        patch_dct = cv2.log(cv2.dct(patch.astype(np.float32)))
        #m = cv2.moments(patch_dct)
        #x = m['m10']/m['m00']
        #y = m['m01']/m['m00']
        #if (x>0.02 and y>0.02):
        #if (np.any(patch_dct[6:8,0] > 4) and np.any(patch_dct[0,3:6] > 4)):
        if (np.sum(patch_dct[5:8,0]) > 11 and np.sum(patch_dct[0,3:6]) > 9):
          index = np.floor(img_patch[...,0]/8)+32*np.floor(img_patch[...,1]/8)+32*32*np.floor(img_patch[...,2]/8)
          out = np.argmax(self.mat['w2c'][np.int16(index)],2)
          mask[i*40:(i+1)*40,j*40:(j+1)*40] = cv2.inRange(out,5,5)

    if self.disp_img == None:
      return

    masked_disp_img = np.multiply(self.disp_img,mask/255).astype(np.float32)
    self.disp_out = DisparityImage()
    self.disp_out.min_disparity = self.min_disparity
    self.disp_out.max_disparity = self.max_disparity
    self.disp_out.image = self.bridge.cv_to_imgmsg(cv2.cv.fromarray(masked_disp_img))
    # For some reason, cv_bridge is setting the step to 640, not 4*640
    # fixing this manually
    #self.disp_out.image.step = 2560
    self.disp_out.header = self.header

    masked_img = np.multiply(img, (mask[...,None]/255))
    img_out = self.bridge.cv_to_imgmsg(cv2.cv.fromarray(masked_img),'rgb8')
    img_out.header = Image.header

    self.img_pub.publish(img_out)
    self.disp_pub.publish(self.disp_out)

  def handle_disp(self,DisparityImage):
    self.disp_img = np.asarray(self.bridge.imgmsg_to_cv(DisparityImage.image))
    self.min_disparity = DisparityImage.min_disparity
    self.max_disparity = DisparityImage.max_disparity
    self.header = DisparityImage.header

if __name__=="__main__":
  try:
    freq_fence_detection()
    rospy.spin()
  except rospy.ROSInterruptException: pass
