#!/usr/bin/env python
PKG = 'samplereturn'
import roslib; roslib.load_manifest(PKG)
from rosbag import Bag

inbag_string = '/home/zlizer/Desktop/2012-06-02-17-46-30_moline_fixed.bag'
outbag_string = '/home/zlizer/Desktop/2012-06-02-17-46-30_moline_fixed_odom.bag'

outbag = Bag(outbag_string,'w')
for topic, msg, t in Bag(inbag_string):
  if topic == 'tf':
    if msg.transforms[0].header.frame_id == '/odom':
      outbag.write(topic,msg,t)
  else:
    outbag.write(topic,msg,t)
