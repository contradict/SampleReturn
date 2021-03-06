#!/usr/bin/env python
import copy
import roslib
import rospy
import numpy as np
import cv2
import yaml

from sensor_msgs.msg import Image,CameraInfo,PointCloud2
from stereo_msgs.msg import DisparityImage
from geometry_msgs.msg import Vector3Stamped,Point,PointStamped
from samplereturn_msgs.msg import NamedPoint
from cv_bridge import CvBridge
from tf import TransformListener
import scipy.io
import scipy.linalg
import threading
import Queue

class color_name_sample_detection(object):

  def __init__(self):
    rospy.init_node('color_name_sample_detection',log_level=rospy.INFO)
    self.monocular_img_sub = rospy.Subscriber('monocular_img',Image, queue_size=1,callback=self.handle_monocular_img, buff_size=60000000)
    self.cam_info_sub = rospy.Subscriber('cam_info',CameraInfo, queue_size=1, callback=self.handle_info)
    #I think this node only uses the search camera - JL
    #self.left_img_sub = rospy.Subscriber('left_img',Image, queue_size=1,callback=self.handle_left_img)
    #self.right_img_sub = rospy.Subscriber('right_img',Image, queue_size=1,callback=self.handle_right_img)
    #self.disp_sub = rospy.Subscriber('disp',DisparityImage, queue_size=1,callback=self.handle_disp)

    self.bridge = CvBridge()
    self.tf = TransformListener()

    maxArea = rospy.get_param('~max_area',800.0)
    minArea = rospy.get_param('~min_area',100.0)
    delta = rospy.get_param('~delta',10.0)

    self.min_size = rospy.get_param('~min_size',0.08)
    self.max_size = rospy.get_param('~max_size',0.4)

    self.mser = cv2.MSER()
    self.mser.setDouble('maxArea', maxArea)
    self.mser.setDouble('minArea', minArea)
    self.mser.setDouble('delta', delta)

    if rospy.get_param('~enable_debug',True):
      debug_img_topic = 'debug_img'
      self.debug_img_pub = rospy.Publisher(debug_img_topic, Image)

    self.named_img_point_pub = rospy.Publisher('named_img_point', NamedPoint)
    self.named_point_pub = rospy.Publisher('named_point', NamedPoint)

    color_file = rospy.get_param('~color_file')
    self.color_mat = scipy.io.loadmat(color_file)
    self.color_names = ['black','blue','brown','gray','green','orange','pink','purple','red','white','yellow']
    #self.sample_names = [None,None,'wood_block','pre_cached',None,'orange_pipe','pink_ball',None,'red_puck','pre_cached','yellow_rock']
    self.sample_names = [None,'pre_cached',None,'pre_cached',None,None,None,None,None,'pre_cached','pre_cached']
    self.sample_thresh = [None,0.1,0.1,0.1,None,0.1,0.1,None,0.1,0.1,0.1]

    self.min_disp = 0.0
    self.max_disp = 128.0

    self.disp_img = None

    self.static_mask = None

    self.nthreads = 4
    self.height = None
    self.threads = []
    self.q_img = Queue.Queue()
    self.q_proj = Queue.Queue()

  def handle_left_img(self,Image):
    detections = self.find_samples(Image)
    self.debug_img_pub.publish(self.bridge.cv_to_imgmsg(cv2.cv.fromarray(self.debug_img),'bgr8'))

  def handle_right_img(self, Image):
    detections = self.find_samples(Image)

  def handle_monocular_img(self, Image):
    detections = self.find_samples_threaded(Image)
    self.debug_img_pub.publish(self.bridge.cv_to_imgmsg(cv2.cv.fromarray(self.debug_img),'bgr8'))

  def threaded_mser(self, img, header):
    for i in range(self.nthreads):
      t = threading.Thread(
          target=self.split_mser_and_color,
          args=(img[self.height*i:self.height*(i+1),:],self.static_mask[self.height*i:self.height*(i+1),:],
            i,self.q_img,self.q_proj,header))
      self.threads.append(t)
      t.start()
    for t in self.threads:
      t.join()

  def split_mser_and_color(self, img, mask, i, q_img, q_proj, header):
    regions = self.mser.detect(img, mask)
    for r in regions:
      r += np.array([0,self.height*i])
    hulls = [cv2.convexHull(r.reshape(-1,1,2)) for r in regions]
    img = cv2.cvtColor(self.img,cv2.COLOR_BGR2RGB)
    mask = np.zeros((img.shape[0],img.shape[1]))
    for h in (hulls):
      top_index, similarity = self.compute_color_name(h,img,mask)
      if self.sample_names[top_index] is not None and similarity >= self.sample_thresh[top_index]:
        moments = cv2.moments(h)
        # converts to x,y
        location = np.array([moments['m10']/moments['m00'],moments['m01']/moments['m00']])
        named_img_point = NamedPoint()
        named_img_point.header = copy.deepcopy(header)
        named_img_point.point.x = location[0]
        named_img_point.point.y = location[1]
        named_img_point.name = self.sample_names[top_index]

        named_point = self.project_xyz_point(h, top_index, header)
        #rospy.logdebug("Named_point: %s",named_point)

        size = self.real_size_check(h,header)
        if self.min_size < size < self.max_size:
          q_img.put(named_img_point)
          q_proj.put(named_point)

  def find_samples_threaded(self, Image):
    self.img = np.asarray(self.bridge.imgmsg_to_cv(Image,'bgr8'))
    self.debug_img = self.img.copy()

    if self.static_mask is None:
      self.static_mask = np.zeros((self.img.shape[0],self.img.shape[1],1),np.uint8)
      self.static_mask[600:,:,:] = 1
    if self.height is None:
      self.height = self.img.shape[0]/self.nthreads

    gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
    flip = 255 - hsv[:,:,1]
    test = gray.astype(np.float32)*flip.astype(np.float32)
    test /= np.max(test)
    test = (test*255).astype(np.uint8)
    self.threaded_mser(test, Image.header)

    while not self.q_img.empty():
      self.named_img_point_pub.publish(self.q_img.get())
    while not self.q_proj.empty():
      pt = self.q_proj.get()
      self.named_point_pub.publish(pt)

  def find_samples(self, Image):
    self.img = np.asarray(self.bridge.imgmsg_to_cv(Image,'bgr8'))
    self.debug_img = self.img.copy()

    #if self.static_mask is None:
    #  self.static_mask = np.zeros((self.img.shape[0],self.img.shape[1],1),np.uint8)
    #  self.static_mask[400:,:,:] = 1

    lab = cv2.cvtColor(self.img, cv2.COLOR_BGR2LAB)
    gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
    #a_regions = self.mser.detect(lab[:,:,1] ,None)
    #a_hulls = [cv2.convexHull(r.reshape(-1,1,2)) for r in a_regions]
    #b_regions = self.mser.detect(lab[:,:,2] ,None)
    #b_hulls = [cv2.convexHull(r.reshape(-1,1,2)) for r in b_regions]
    #g_regions = self.mser.detect(gray, self.static_mask)
    mask = np.ones((self.img.shape[0],self.img.shape[1],1),np.uint8)
    mask[:,:100] = 0
    #mask[:,-100:] = 0
    g_regions = self.mser.detect(gray, mask)
    rospy.logdebug("number of regions: %s", len(g_regions))
    g_hulls = [cv2.convexHull(r.reshape(-1,1,2)) for r in g_regions]
    rospy.logdebug("number of hulls: %s", len(g_hulls))
    img = cv2.cvtColor(self.img,cv2.COLOR_BGR2RGB)
    mask = np.zeros((img.shape[0],img.shape[1]))
    #for h in (a_hulls + b_hulls):
    for h in (g_hulls):
      top_index, similarity = self.compute_color_name(h,img,mask)
      if self.sample_names[top_index] is not None and similarity >= self.sample_thresh[top_index]:
        moments = cv2.moments(h)
        # converts to x,y
        location = np.array([moments['m10']/moments['m00'],moments['m01']/moments['m00']])
        rospy.logdebug("Publishing Named Point")
        named_img_point = NamedPoint()
        named_img_point.header = Image.header
        named_img_point.point.x = location[0]
        named_img_point.point.y = location[1]
        named_img_point.name = self.sample_names[top_index]
        self.named_img_point_pub.publish(named_img_point)
        named_point = self.project_xyz_point(h, top_index, Image.header)
        self.named_point_pub.publish(named_point)

  def project_xyz_point(self, hull, top_index, header):
    if self.disp_img is not None:
      rect = cv2.boundingRect(hull)
      local_disp = self.disp_img[rect[1]:rect[1]+rect[3],rect[0]:rect[0]+rect[2]]
      # Trim off extreme disparities
      local_disp = cv2.threshold(local_disp.astype(np.float32), self.min_disp, 0, cv2.THRESH_TOZERO)[1]
      local_disp = cv2.threshold(local_disp.astype(np.float32), self.max_disp, 0, cv2.THRESH_TOZERO_INV)[1]
      # Sort disparities, grab ends, compute mean
      count = cv2.countNonZero(local_disp)
      local_disp = local_disp.reshape((rect[2]*rect[3],1))
      local_disp = np.sort(local_disp)
      accum_disp = local_disp[:10].sum() + local_disp[count-10:count].sum()
      ave_disp = accum_disp/20.
      # Depth in meters
      ave_depth = self.f*self.T/ave_disp
      x = rect[0]+rect[2]/2
      y = rect[1]+rect[3]/2
      XY = np.dot(self.inv_K,np.array([x,y,1]))
      XYZ = XY*ave_depth
      named_point = NamedPoint()
      named_point.name = self.sample_names[top_index]
      named_point.header = copy.deepcopy(header)
      named_point.point.x = XYZ[0]
      named_point.point.y = XYZ[1]
      named_point.point.z = XYZ[2]
      #self.named_point_pub.publish(named_point)
      return named_point
    else:
      rect = cv2.boundingRect(hull)
      x = (rect[0]+rect[2]/2)#*2.
      y = (rect[1]+rect[3]/2)#*2.
      XY = np.dot(self.inv_K,np.array([x,y,1]))
      named_point = NamedPoint()
      named_point.name = self.sample_names[top_index]
      named_point.header = copy.deepcopy(header)
      named_point.point.x = XY[0]
      named_point.point.y = XY[1]
      named_point.point.z = 1.0
      return named_point
      #self.named_point_pub.publish(named_point)


  def handle_disp(self,DisparityImage):
    self.disp_img = np.asarray(self.bridge.imgmsg_to_cv(DisparityImage.image))
    self.disp_header = copy.deepcopy(DisparityImage.header)
    self.min_disparity = DisparityImage.min_disparity
    self.max_disparity = DisparityImage.max_disparity
    self.f = DisparityImage.f
    self.T = DisparityImage.T

  def handle_info(self, CameraInfo):
    # grab camera matrix and distortion model
    self.K = CameraInfo.K
    self.D = CameraInfo.D
    self.R = CameraInfo.R
    self.P = CameraInfo.P
    self.h = CameraInfo.height
    self.w = CameraInfo.width
    self.frame_id = CameraInfo.header.frame_id
    self.P = np.asarray(self.P).reshape(3,4)
    self.K = np.asarray(self.K).reshape(3,3)
    self.inv_K = scipy.linalg.inv(self.K)

  def compute_color_name(self,hull,img,mask):
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
    #if similarity > 0.1 and (top_index == 3 or top_index == 9):
    cv2.putText(self.debug_img,self.color_names[top_index] + str(similarity),(rect[0],rect[1]),cv2.FONT_HERSHEY_PLAIN,2,(255,0,255))
    if top_index == 3 or top_index == 9 or top_index == 1:
      cv2.polylines(self.debug_img,[hull],1,(255,0,255),3)
      cv2.putText(self.debug_img,self.color_names[top_index] + str(similarity),(rect[0],rect[1]),cv2.FONT_HERSHEY_PLAIN,2,(255,0,255))
    return top_index,similarity

  def cast_ray(self, point_in, tf, name):
    #rospy.logdebug("Point In: %s", point_in)

    base_link_point = tf.transformPoint('/base_link', point_in)
    t = tf.getLatestCommonTime('/base_link', point_in.header.frame_id)
    pos, quat = tf.lookupTransform('/base_link', point_in.header.frame_id, t)
    height = pos[2]
    #rospy.logdebug("Pos: %s", pos)

    x_slope = np.abs((pos[0]-base_link_point.point.x)/(pos[2]-base_link_point.point.z))
    y_slope = np.abs((pos[1]-base_link_point.point.y)/(pos[2]-base_link_point.point.z))
    #rospy.logdebug("X Slope: %s", x_slope)
    #rospy.logdebug("Y Slope: %s", y_slope)

    ground_point = np.array([0.,0.,0.])
    ground_point[0] = x_slope*height
    ground_point[1] = y_slope*height

    ground_named_point = NamedPoint()
    ground_named_point.point.x = ground_point[0]
    ground_named_point.point.y = ground_point[1]
    ground_named_point.point.z = ground_point[2]
    ground_named_point.header = copy.deepcopy(point_in.header)
    ground_named_point.header.frame_id = 'base_link'
    ground_named_point.name = name

    odom_named_point = self.tf.transformPoint('/odom',ground_named_point)

    #rospy.logdebug("Ground Point: %s", ground_named_point)
    #rospy.logdebug("Odom Point: %s", odom_named_point)

    return ground_named_point, odom_named_point

  def real_size_check(self,hull,header):
    rect = cv2.boundingRect(hull)
    #top_left = np.array([rect[0],rect[1]])
    bot_left = np.array([rect[0],rect[1]+rect[3]])
    bot_right = np.array([rect[0]+rect[2],rect[1]+rect[3]])

    #rospy.logdebug("Top Left: %s", top_left)
    #rospy.logdebug("Bot Right: %s", bot_right)

    bot_left_point = PointStamped()
    bot_left_point.header = copy.deepcopy(header)
    bot_left_point.point.x = bot_left[0]
    bot_left_point.point.y = bot_left[1]
    bot_left_point.point.z = 1.0
    bot_right_point = PointStamped()
    bot_right_point.header = copy.deepcopy(header)
    bot_right_point.point.x = bot_right[0]
    bot_right_point.point.y = bot_right[1]
    bot_right_point.point.z = 1.0

    #rospy.logdebug("Top Left Point: %s", top_left_point)
    #rospy.logdebug("Bot Right Point: %s", bot_right_point)

    bot_left_ground, bot_left_odom = self.cast_ray(bot_left_point,self.tf,'bot_left')
    bot_right_ground, bot_right_odom = self.cast_ray(bot_right_point,self.tf,'bot_right')

    #rospy.logdebug("Top Left Ground: %s", top_left_ground)
    #rospy.logdebug("Bot Right Ground: %s", bot_right_ground)

    width = np.array([0.,0.])
    width[0] = bot_left_ground.point.x - bot_right_ground.point.x
    width[1] = bot_left_ground.point.y - bot_right_ground.point.y
    rospy.logdebug("Width: %s", width)
    size = scipy.linalg.norm(width)
    rospy.logdebug("Size: %s", size)
    return size

if __name__=="__main__":
  try:
    color_name_sample_detection()
    rospy.spin()
  except rospy.ROSInterruptException: pass
