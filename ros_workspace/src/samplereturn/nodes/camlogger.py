#!/usr/bin/env python
import roslib; roslib.load_manifest('samplereturn')
from photo.srv import *
from sensor_msgs.msg import Image,CameraInfo
import rospy
import yaml

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

def parse_yaml(filename):
  stream = file(filename, 'r')
  calib_data = yaml.load(stream)
  cam_info = CameraInfo()
  cam_info.width = calib_data['image_width']
  cam_info.height = calib_data['image_height']
  cam_info.K = calib_data['camera_matrix']['data']
  cam_info.D = calib_data['distortion_coefficients']['data']
  cam_info.R = calib_data['rectification_matrix']['data']
  cam_info.P = calib_data['projection_matrix']['data']
  cam_info.distortion_model = calib_data['distortion_model']
  return cam_info

def camlogger():
  topic = 'cam_img'
  info_topic = 'cam_info'
  pub = rospy.Publisher(topic,Image)
  info_pub = rospy.Publisher(info_topic,CameraInfo)
  cam_info = parse_yaml('')
  rospy.init_node('camlogger',log_level=rospy.DEBUG)
  frame_id = rospy.get_param('~frame_id')
  cam_info.header.frame_id = frame_id
  seq_id = 0
  rate = rospy.get_param('~rate')
  r = rospy.Rate(rate)
  while not rospy.is_shutdown():
    img = capture_image()
    rospy.logdebug("img (%d, %d)", img.width, img.height)
    img.header.stamp = rospy.Time.now()
    cam_info.header.stamp = rospy.Time.now()
    img.header.frame_id = frame_id
    img.header.seq = seq_id
    cam_info.header.seq = seq_id
    pub.publish(img)
    info_pub.publish(cam_info)
    seq_id += 1
    r.sleep()

if __name__=="__main__":
  try:
    camlogger()
  except rospy.ROSInterruptException: pass
