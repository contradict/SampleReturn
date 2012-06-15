#!/usr/bin/env python
# This parses a rosbag for camera messages and rewrites their frame_id

PKG = 'sensor_msgs'
import roslib; roslib.load_manifest(PKG)
import rospy
#import rosrecord
import rosbag
import fileinput
import os

def fix(inbags):
  for b in inbags:
    print "Trying to migrating file: %s"%b
    outbag = b+'.tmp'
    rebag = rosrecord.Rebagger(outbag)
    try:
      for (topic, msg, t) in rosrecord.logplayer(b, raw=False):
        if msg._type == 'sensor_msgs/CameraInfo' or msg._type == 'sensor_msgs/Image':
          msg.header.frame_id = 'l_forearm_cam_optical_frame'
          rebag.add(topic, msg, t, raw=False)
        else:
          rebag.add(topic, msg, t, raw=False)
      rebag.close()
    except rosrecord.ROSRecordException, e:
      print " Migration failed: %s"%(e.message)
      os.remove(outbag)
      continue

    oldnamebase = b+'.old'
    oldname = oldnamebase
    i = 1
    while os.path.isfile(oldname):
      i=i+1
      oldname = oldnamebase + str(i)
    os.rename(b, oldname)
    os.rename(outbag, b)
    print " Migration successful.  Original stored as: %s"%oldname

if __name__ == '__main__':
  import sys
  if len(sys.argv) >= 2:
    fix(sys.argv[1:])
  else:
    print "usage: %s bag1 [bag2 bag3 ...]" % sys.argv[0]
