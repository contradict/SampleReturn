#!/usr/bin/env python
PKG = 'samplereturn'
import roslib; roslib.load_manifest(PKG)
from rosbag import Bag
import yaml

inbag_string = '/home/zlizer/Desktop/2012-06-02-17-52-23_moline.bag'
outbag_string = '/home/zlizer/Desktop/2012-06-02-17-52-23_moline_fixed.bag'

leftcalibpath = '../ros/samplereturn/calibration/ffmv-manipulator-cheap4mm-left.yaml'
rightcalibpath = '../ros/samplereturn/calibration/ffmv-manipulator-cheap4mm-right.yaml'

leftstream = open(leftcalibpath,'r')
rightstream = open(rightcalibpath,'r')
leftcalib = yaml.load(leftstream)
rightcalib = yaml.load(rightstream)

outbag = Bag(outbag_string,'w')
for topic, msg, t in Bag(inbag_string):
  if topic == 'tf':
    if msg.transforms[0].header.frame_id == '/odom':
      outbag.write(topic,msg,t)
  if msg._type == 'sensor_msgs/CameraInfo':
    if msg.header.frame_id == '/manipulator_left_camera':
      msg.distortion_model = leftcalib['distortion_model']
      msg.height = leftcalib['image_height']
      msg.width = leftcalib['image_width']
      msg.D = leftcalib['distortion_coefficients']['data']
      msg.K = leftcalib['camera_matrix']['data']
      msg.R = leftcalib['rectification_matrix']['data']
      msg.P = leftcalib['projection_matrix']['data']
      outbag.write(topic,msg,t)
    if msg.header.frame_id == '/manipulator_right_camera':
      msg.header.frame_id = '/manipulator_left_camera'
      msg.height = rightcalib['image_height']
      msg.width = rightcalib['image_width']
      msg.distortion_model = rightcalib['distortion_model']
      msg.D = rightcalib['distortion_coefficients']['data']
      msg.K = rightcalib['camera_matrix']['data']
      msg.R = rightcalib['rectification_matrix']['data']
      msg.P = rightcalib['projection_matrix']['data']
      outbag.write(topic,msg,t)
  else:
    outbag.write(topic,msg,t)

outbag.close()
