#!/usr/bin/env python
import roslib
import rospy
import numpy as np
import cv2
import yaml

from sensor_msgs.msg import Image,CameraInfo,PointCloud2
from stereo_msgs.msg import DisparityImage
from geometry_msgs.msg import Vector3Stamped,Point,PointStamped
from cv_bridge import CvBridge
from tf import TransformListener

class sample_detection(object):
  def __init__(self):
    rospy.init_node('sample_detection',anonymous=True)
    self.mono_img_sub = rospy.Subscriber('mono_img',Image, self.handle_mono_img)
    self.left_img_sub = rospy.Subscriber('left_img',Image, self.handle_left_img)
    self.right_img_sub = rospy.Subscriber('right_img',Image, self.handle_right_img)
    self.disp_sub = rospy.Subscriber('disp',DisparityImage, self.handle_disp)
    self.cam_info_sub = rospy.Subscriber('cam_info',CameraInfo, self.handle_info)
    self.tf_listener = TransformListener()

    self.bridge = CvBridge()

    sample_file = rospy.get_param('~samples')
    stream = file(sample_file,'r')
    self.samples = yaml.load(stream)

    self.mser = cv2.MSER()

    if rospy.get_param('~enable_debug',True):
      debug_img_topic = 'debug_img'
      self.debug_img_pub = rospy.Publisher(debug_img_topic, Image)

    self.sample_disparity ={}
    self.sample_points ={}
    self.sample_imgpoints ={}
    for s in self.samples:
      topic = s + '_pointstamped'
      self.sample_points[s] = rospy.Publisher(topic, PointStamped)
      topic = s + '_imgpoints'
      self.sample_imgpoints[s] = rospy.Publisher(topic, Point)
      topic = s + '_disparity'
      self.sample_disparity[s] = rospy.Publisher(topic, DisparityImage)


  def handle_mono_img(self, Image):
    detections = self.find_samples(Image)
    self.debug_img_pub.publish(self.bridge.cv_to_imgmsg(cv2.cv.fromarray(self.debug_img),'bgr8'))
    for d in detections:
      if detections[d]['location'] is None:
        continue
      else:
        location = detections[d]['location']
        self.sample_imgpoints[d].publish(x=location[0],y=location[1])#,header.stamp)
        # Intersects a camera ray with a flat ground plane
        #self.project_centroid(location)

  def handle_left_img(self, Image):
    detections = self.find_samples(Image)
    for d in detections:
      if detections[d]['location'] is None:
        continue
      else:
        location = detections[d]['location']
        self.sample_imgpoints[d].publish(x=location[0],y=location[1])
        # For now, only publish the left image as a debug
        self.debug_img_pub.publish(self.bridge.cv_to_imgmsg(cv2.cv.fromarray(self.debug_img),'bgr8'))
        # Grab associated part of disparity image
        mask = np.zeros_like(np.asarray(self.bridge.imgmsg_to_cv(Image,'bgr8')))
        cv2.drawContours(mask,[detections[d]['hull']],-1,(255,255,255),-1)
        mask = mask[:,:,0]
        masked_disp_img = cv2.multiply(mask.astype(np.float32),self.disp_img).astype(np.float32)
        disp_out = DisparityImage()
        disp_out.min_disparity = self.min_disparity
        disp_out.max_disparity = self.max_disparity
        disp_out.image = self.bridge.cv_to_imgmsg(cv2.cv.fromarray(masked_disp_img))
        disp_out.image.step = 2560
        disp_out.header = self.disp_header
        self.sample_disparity[d].publish(disp_out)

  def handle_right_img(self, Image):
    detections = self.find_samples(Image)
    for d in detections:
      if detections[d]['location'] is None:
        continue
      else:
        location = detections[d]['location']
        self.sample_imgpoints[d].publish(x=location[0],y=location[1])
        # Grab associated part of disparity image
        mask = np.zeros_like(np.asarray(self.bridge.imgmsg_to_cv(Image,'bgr8')))
        cv2.drawContours(mask,[detections[d]['hull']],-1,(255,255,255),-1)
        mask = mask[:,:,0]
        masked_disp_img = cv2.multiply(mask.astype(np.float32),self.disp_img).astype(np.float32)
        disp_out = DisparityImage()
        disp_out.min_disparity = self.min_disparity
        disp_out.max_disparity = self.max_disparity
        disp_out.image = self.bridge.cv_to_imgmsg(cv2.cv.fromarray(masked_disp_img))
        disp_out.image.step = 2560
        disp_out.header = self.disp_header
        self.sample_disparity[d].publish(disp_out)

  def handle_disp(self,DisparityImage):
    self.disp_img = np.asarray(self.bridge.imgmsg_to_cv(DisparityImage.image))
    self.disp_header = DisparityImage.header
    self.min_disparity = DisparityImage.min_disparity
    self.max_disparity = DisparityImage.max_disparity

  def find_samples(self, Image):
    self.img = np.asarray(self.bridge.imgmsg_to_cv(Image,'bgr8'))
    self.debug_img = self.img.copy()
    lab = cv2.cvtColor(self.img, cv2.COLOR_BGR2LAB)
    a_regions = self.mser.detect(lab[:,:,1] ,None)
    a_hulls = [cv2.convexHull(r.reshape(-1,1,2)) for r in a_regions]
    b_regions = self.mser.detect(lab[:,:,2] ,None)
    b_hulls = [cv2.convexHull(r.reshape(-1,1,2)) for r in b_regions]
    detections = {}
    for s in self.samples:
      detections[s] = {}
      detections[s]['min_dist'] = self.samples[s]['min_dist']
      detections[s]['location'] = None
      if self.samples[s]['channel'] == 'a':
        for h in a_hulls:
          mean = self.compute_color_mean(h,self.img,'lab').astype(np.float32)
          cols = self.samples[s]['covariance']['cols']
          rows = self.samples[s]['covariance']['rows']
          model_covariance = np.asarray(self.samples[s]['covariance']['data'],np.float32).reshape(rows,cols)
          dist = cv2.Mahalanobis(mean,np.asarray(self.samples[s]['mean'],np.float32),model_covariance)
          if dist < detections[s]['min_dist']:
            detections[s]['min_dist'] = dist
            moments = cv2.moments(h)
            # converts to x,y
            location = np.array([moments['m10']/moments['m00'],moments['m01']/moments['m00']])
            detections[s]['location'] = location
            detections[s]['hull'] = h
            cv2.polylines(self.debug_img,[h],1,(255,0,255),3)
            cv2.putText(self.debug_img,s,(int(location[0]),int(location[1])),cv2.FONT_HERSHEY_PLAIN,2,(0,255,255))
      elif self.samples[s]['channel'] == 'b':
        for h in b_hulls:
          mean = self.compute_color_mean(h,self.img,'lab').astype(np.float32)
          cols = self.samples[s]['covariance']['cols']
          rows = self.samples[s]['covariance']['rows']
          model_covariance = np.asarray(self.samples[s]['covariance']['data'],np.float32).reshape(rows,cols)
          dist = cv2.Mahalanobis(mean,np.asarray(self.samples[s]['mean'],np.float32),model_covariance)
          if dist < detections[s]['min_dist']:
            detections[s]['min_dist'] = dist
            moments = cv2.moments(h)
            # converts to x,y
            location = np.array([moments['m10']/moments['m00'],moments['m01']/moments['m00']])
            detections[s]['location'] = location
            detections[s]['hull'] = h
            cv2.polylines(self.debug_img,[h],1,(255,255,0),3)
            cv2.putText(self.debug_img,s,(int(location[0]),int(location[1])),cv2.FONT_HERSHEY_PLAIN,2,(0,255,255))
    return detections

  def handle_info(self, CameraInfo):
    # grab camera matrix and distortion model
    self.K = CameraInfo.K
    self.D = CameraInfo.D
    self.R = CameraInfo.R
    self.P = CameraInfo.P
    self.h = CameraInfo.height
    self.w = CameraInfo.width
    self.frame_id = CameraInfo.header.frame_id

  def project_centroid(self,centroid):
    # project image coordinates into ray from camera, intersect with ground plane
    point = np.zeros((1,1,2))
    point[0,0,0] = centroid[0]
    point[0,0,1] = centroid[1]
    rospy.logerr(np.asarray(self.K).reshape((3,3)))
    rect_point = cv2.undistortPoints(point,np.asarray(self.K).reshape((3,3)),np.asarray(self.D))
    rospy.logerr(rect_point)
    x = rect_point[0,0,0]
    y = rect_point[0,0,1]
    r = np.sqrt(x**2 + y**2)
    theta = np.arctan(r)
    phi = np.arctan2(y,x)
    #self.tf_listener.lookupTransform('/base_link',self.frame_id)
    self.tf_listener.transform('/base_link',self.frame_id)

  def compute_color_mean(self,hull,img,color_space):
    mask = np.zeros_like(img)
    cv2.drawContours(mask,[hull],-1,(255,255,255),-1)
    if color_space == 'rgb':
      mean = np.asarray(cv2.mean(img,mask[:,:,0])[:3])
      return mean
    elif color_space == 'lab' or color_space == 'hsv':
      mean = np.asarray(cv2.mean(img,mask[:,:,0])[1:3])
      return mean


if __name__=="__main__":
  try:
    sample_detection()
    rospy.spin()
  except rospy.ROSInterruptException: pass
