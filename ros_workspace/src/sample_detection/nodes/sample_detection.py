import numpy as np
import cv2
import yaml

from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import Vector3Stamped
from cv_bridge import CvBridge
from tf import TransformListener
import rospy

class sample_detection(object):
  def __init__():
    rospy.init_node('sample_detection',anonymous=True)
    self.img_sub = rospy.Subscriber('cam_img',Image, self.find_samples)
    self.cam_info_sub = rospy.Subcriber('cam_info',CameraInfo, self.handle_info)
    self.tf = TransformListener()

    self.bridge = CvBridge()
    self.mser = cv2.MSER()

    sample_file = rospy.get_param('samples')
    stream = file(sample_file,'r')
    self.samples = yaml.load(stream)

    if rospy.get_param('enable_debug'):
      debug_img_topic = 'debug_img'
      self.debug_img_pub = rospy.Publisher(debug_img_topic, Image)

    self.sample_points ={}
    for s in self.samples:
      topic = s + '_pointcloud'
      self.sample_points[topic] = rospy.Publisher(topic, PointCloud2)


  def find_samples(self, Image):
    self.img = np.asarray(self.bridge.imgmsg_to_cv(Image,'bgr8'))
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    a_regions = mser.detect(lab[:,:,1] ,None)
    a_hulls = [cv2.convexHull(r.reshape(-1,1,2)) for r in a_regions]
    b_regions = mser.detect(lab[:,:,2] ,None)
    b_hulls = [cv2.convexHull(r.reshape(-1,1,2)) for r in b_regions]
    for s in self.samples:
      if s['channel'] == 'a':
        for h in a_hulls:
          self.compute_color_model(h)
      elif s['channel'] == 'b':
        for h in b_hulls
          self.compute_color_model(h)

  def handle_info(self, CameraInfo):
    # grab camera matrix and distortion model
    self.K = CameraInfo.K
    self.D = CameraInfo.D

  def project_regions(self,centroid,size):

  def compute_color_model(self,hull):
    r = cv2.boundingRect(hull)

  def draw_debug(self):

if __name__=="__main__":
  try:
    sample_detection()
    rospy.spin()
  except rospy.ROSInterruptException: pass
