#!/usr/bin/env python
import roslib; roslib.load_manifest('samplereturn')
from photo.srv import *
from sensor_msgs.msg import Image,CameraInfo
import rospy
import yaml

import std_msgs.msg as std_msg

paused=False

def pause(msg):
    global paused
    paused = msg.data
    rospy.logdebug("paused: %s", paused)

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
  rospy.init_node('camlogger')

  topic = 'cam_img'
  info_topic = 'cam_info'
  pub = rospy.Publisher(topic,Image)
  info_pub = rospy.Publisher(info_topic,CameraInfo)

  rospy.Subscriber('/pause_state', std_msg.Bool, pause)

  cam_info = CameraInfo()
  calib_file = rospy.get_param('~calib_file', None)
  frame_id = rospy.get_param('~frame_id', '/search_camera_lens')
  rospy.logdebug("calib_file: %s", calib_file)
  if calib_file is not None:
    cam_info = parse_yaml(calib_file)
  cam_info.header.frame_id = frame_id

  seq_id = 0
  rate = rospy.get_param('~rate',1.0)
  r = rospy.Rate(rate)
  while not rospy.is_shutdown():
    if not paused:
      now = rospy.Time.now()

      img = capture_image()
      rospy.logdebug("img (%d, %d)", img.width, img.height)

      img.header.stamp = now
      img.header.frame_id = frame_id
      img.header.seq = seq_id
      pub.publish(img)

      cam_info.header.stamp = now
      cam_info.header.seq = seq_id
      info_pub.publish(cam_info)

      seq_id += 1
    else:
        rospy.logdebug("paused")
    r.sleep()

if __name__=="__main__":
  try:
    camlogger()
  except rospy.ROSInterruptException: pass
