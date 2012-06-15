#!/usr/bin/env python
import roslib; roslib.load_manifest('samplereturn')
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3Stamped
import rospy
import numpy as np
from cv_bridge import CvBridge
import cv2
import scipy.linalg

class objectdetector(object):
  def __init__(self,I,target_mean,target_cov,thresh,objects):#,D,target):#,cov,dist):
    # Set up the subscription and publisher
    rospy.init_node('objectdetector',anonymous=True)
    self.img_sub = rospy.Subscriber('camimg',Image,self.findobjects)
    vec_topic = 'objectvector'
    self.vec_pub = rospy.Publisher(vec_topic, Vector3Stamped)
    img_topic = 'objimg'
    self.img_pub = rospy.Publisher(img_topic, Image)
    img_det_topic = 'objdetimg'
    self.img_det_pub = rospy.Publisher(img_det_topic, Image)
    self.I = I
    self.I_inv = scipy.linalg.inv(I)
    self.mean = target_mean#.astype(np.float32)
    self.cov = target_cov#.astype(np.float32)
    self.thresh = thresh
    self.objects = objects

  def findobjects(self,Image):
    # Searches for colored objects in an Image msg
    bridge = CvBridge()
    img = np.asarray(bridge.imgmsg_to_cv(Image,'bgr8'))
    img_det = img.copy()
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    a_chan = lab[:,:,1].copy()
    mser = cv2.MSER()
    bins = 10
    regions = mser.detect(a_chan, None)
    hulls = [cv2.convexHull(p.reshape(-1,1,2)) for p in regions]
    best_match = {}
    best_dist = {}
    for hull in hulls:
      dist = {}
      count = 0
      acc = np.array([0,0,0])
      r = cv2.boundingRect(hull)
      cv2.rectangle(img,(r[0],r[1]),(r[0]+r[2],r[1]+r[3]),(0,0,255),2)
      weights = np.zeros((r[3],r[2]))
      for i in range(r[3]):
        for j in range(r[2]):
          if cv2.pointPolygonTest(hull,(i,j),False):
            acc += lab[r[1]+i,r[0]+j,:]
            count += 1
      for ob in objects:
        dist[ob] = cv2.Mahalanobis((acc/count).astype(np.float32),self.mean[ob].astype(np.float32),self.cov[ob].astype(np.float32))
      best_ob = dist.keys()[np.argmin(dist.values())]
      print best_ob,dist[best_ob]
      if not best_match.has_key(best_ob):
        print "New Match"
        best_match[best_ob] = hull
        best_dist[best_ob] = dist[best_ob]
      if dist[best_ob] < best_dist[best_ob]:
        print "New Best"
        best_match[best_ob] = hull
        best_dist[best_ob] = dist[best_ob]
    for ob,hull in best_match.iteritems():
      r = cv2.boundingRect(hull)
      center = np.array([(r[0]+r[2]/2),(r[1]+r[3]/2),1])
      cv2.putText(img_det,ob,(center[0],center[1]),2,1,(255,0,255))
      cv2.polylines(img_det, hull, 1, (255,255,0), 3)
      #dist =  cv2.Mahalanobis((acc/count).astype(np.float32),self.mean,self.cov)
      #  print dist
      #if dist < self.thresh:
      #  print "Found Object!"
      #  center = np.array([(r[0]+r[2]/2),(r[1]+r[3]/2),1])
      #  cv2.putText(img_det,"Red Puck",(center[0],center[1]),2,1,(255,0,255))
      #  ray = np.dot(self.I_inv,center)
      #  ray /= ray[2]
      #  vec = Vector3Stamped()
      #  vec.vector.x = ray[0]
      #  vec.vector.y = ray[1]
      #  vec.vector.z = ray[2]
      #  vec.header.frame_id = 'search_camera'
      #  vec.header.stamp = Image.header.stamp
      #  self.vec_pub.publish(vec)
      #  cv2.polylines(img_det, hull, 1, (255,255,0), 3)
    self.img_det_pub.publish(bridge.cv_to_imgmsg(cv2.cv.fromarray(img_det)))
    cv2.polylines(img, hulls, 1, (0,255,0), 3)
    self.img_pub.publish(bridge.cv_to_imgmsg(cv2.cv.fromarray(img)))


if __name__=="__main__":
  I = np.array([[4284.796, 0, 2624],[0,4289,1677],[0,0,1]])
  #D = np.array([-0.178886 0.145214 0.000428 -0.001064 0.000000])
  objects_path = '/home/zlizer/Desktop/ColorModelImages/'
  #objects = ('red_puck/')
  objects = ('red_puck/','orange_pipe/','pink_sphere/')
  mean = {}
  cov = {}
  icov = {}
  for ob in objects:
    means = np.load(objects_path+ob+'colormeans.npz')
    mean[ob] = means['objacc']/means['count']
    cov[ob] = np.cov(means['objmeans'],rowvar=0)
    icov[ob] = scipy.linalg.inv(cov[ob])
  #means = np.load(objects_path+objects+'colormeans.npz')
  #mean = means['objacc']/means['count']
  #cov = np.cov(means['objmeans'],rowvar=0)
  #icov = scipy.linalg.inv(cov)
  thresh = 2.5

  try:
    objectdetector(I,mean,icov,thresh,objects)#,D,target_hist)#,cov_hist,dist)
    rospy.spin()
  except rospy.ROSInterruptException: pass
