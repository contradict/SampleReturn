#!/usr/bin/env python
import roslib
import rospy
import numpy as np
import cv2
import yaml

from sensor_msgs.msg import Image,CameraInfo,PointCloud2
from stereo_msgs.msg import DisparityImage
from geometry_msgs.msg import Vector3Stamped,Point,PointStamped
from linemod_detector.msg import NamedPoint
from cv_bridge import CvBridge
from tf import TransformListener
import scipy.io

class color_name_sample_detection(object):

  def __init__(self):
    rospy.init_node('color_name_sample_detection',anonymous=True)
    self.left_img_sub = rospy.Subscriber('left_img',Image, queue_size=1,callback=self.handle_left_img)
    self.right_img_sub = rospy.Subscriber('right_img',Image, queue_size=1,callback=self.handle_right_img)
    self.disp_sub = rospy.Subscriber('disparity',DisparityImage, queue_size=1,callback=self.handle_disp)
    self.cam_info_sub = rospy.Subscriber('cam_info',CameraInfo, self.handle_info)

    self.bridge = CvBridge()

    self.mser = cv2.MSER()

    if rospy.get_param('~enable_debug',True):
      debug_img_topic = 'debug_img'
      self.debug_img_pub = rospy.Publisher(debug_img_topic, Image)

    self.named_img_point_pub = rospy.Publisher('named_img_point', NamedPoint)
    self.named_point_pub = rospy.Publisher('named_point', NamedPoint)

    self.color_mat = scipy.io.loadmat('/home/zlizer/Downloads/ColorNaming/w2c.mat')
    self.color_names = ['black','blue','brown','gray','green','orange','pink','purple','red','white','yellow']
    self.sample_names = [None,None,'wood_block','pre_cached',None,'orange_pipe','pink_ball',None,'red_puck','pre_cached','yellow_rock']
    self.sample_thresh = [None,None,0.3,0.3,None,0.4,0.3,None,0.4,0.3,0.3]

  def handle_left_img(self,Image):
    detections = self.find_samples(Image)
    self.debug_img_pub.publish(self.bridge.cv_to_imgmsg(cv2.cv.fromarray(self.debug_img),'bgr8'))

  def handle_right_img(self, Image):
    detections = self.find_samples(Image)

  def find_samples(self, Image):
    self.img = np.asarray(self.bridge.imgmsg_to_cv(Image,'bgr8'))
    self.debug_img = self.img.copy()
    lab = cv2.cvtColor(self.img, cv2.COLOR_BGR2LAB)
    a_regions = self.mser.detect(lab[:,:,1] ,None)
    a_hulls = [cv2.convexHull(r.reshape(-1,1,2)) for r in a_regions]
    b_regions = self.mser.detect(lab[:,:,2] ,None)
    b_hulls = [cv2.convexHull(r.reshape(-1,1,2)) for r in b_regions]
    for h in (a_hulls + b_hulls):
      top_index, similarity = self.compute_color_name(h,self.img)
      if self.sample_names[top_index] is not None and similarity >= self.sample_thresh[top_index]:
        moments = cv2.moments(h)
        # converts to x,y
        location = np.array([moments['m10']/moments['m00'],moments['m01']/moments['m00']])
        named_img_point = NamedPoint()
        named_img_point.header = Image.header
        named_img_point.point.x = location[0]
        named_img_point.point.y = location[1]
        named_img_point.name = self.sample_names[top_index]
        self.named_img_point_pub.publish(named_img_point)

  def handle_disp(self,DisparityImage):
    self.disp_img = np.asarray(self.bridge.imgmsg_to_cv(DisparityImage.image))
    self.disp_header = DisparityImage.header
    self.min_disparity = DisparityImage.min_disparity
    self.max_disparity = DisparityImage.max_disparity

  def handle_info(self, CameraInfo):
    # grab camera matrix and distortion model
    self.K = CameraInfo.K
    self.D = CameraInfo.D
    self.R = CameraInfo.R
    self.P = CameraInfo.P
    self.h = CameraInfo.height
    self.w = CameraInfo.width
    self.frame_id = CameraInfo.header.frame_id

  def compute_color_name(self,hull,img):
    img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
    mask = np.zeros((img.shape[0],img.shape[1]))
    rect = cv2.boundingRect(hull)
    cv2.drawContours(mask,[hull],-1,255,-1)
    small_img = img[rect[1]:rect[1]+rect[3],rect[0]:rect[0]+rect[2]]
    small_mask = mask[rect[1]:rect[1]+rect[3],rect[0]:rect[0]+rect[2]]/255.
    index_img = np.floor(small_img[:,:,0]/8) + 32*np.floor(small_img[:,:,1]/8) + 32*32*np.floor(small_img[:,:,2]/8)
    out = self.color_mat['w2c'][np.int16(index_img)]
    for i in range(out.shape[-1]):
      out[:,:,i] *= small_mask
    ave_vec= np.sum(np.sum(out,axis=0),axis=0)
    top_index = np.argmax(ave_vec)
    similarity = ave_vec[top_index]/cv2.countNonZero(small_mask)
    if similarity > 0.3:
      cv2.putText(self.debug_img,self.color_names[top_index] + str(similarity),(rect[0],rect[1]),cv2.FONT_HERSHEY_PLAIN,2,(255,0,255))
    return top_index,similarity


if __name__=="__main__":
  try:
    color_name_sample_detection()
    rospy.spin()
  except rospy.ROSInterruptException: pass
