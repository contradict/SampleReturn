#!/usr/bin/env python
import roslib
import rospy
import numpy as np
import cv2

from geometry_msgs.msg import Vector3Stamped,Point,PointStamped
from sensor_msgs.msg import PointCloud2
from linemod_detector.msg import NamedPoint
from cv_bridge import CvBridge
from tf import TransformListener

class ray_to_points(object):
  # Alright, this thing is going to take in a ray (in the camera frame) from
  # a monocular camera or a stereo region lacking a disparity match. It's going
  # intersect that ray with a presumed flay ground plane and generate a point
  # cloud around that intersection point that incorporates some of our
  # geometrical uncertainty (most notably, pitch)

  def __init__(self):
    rospy.init_node('ray_to_points',anonymous=True)

    self.named_point_sub = rospy.Subscriber('named_point', NamedPoint, self.handle_named_point)

    self.points_pub = rospy.Publisher('points', PointCloud2)
    self.named_point_pub = rospy.Publisher('point', NamedPoint)

    self.tf = TransformListener()

  def handle_named_point(self, NamedPoint):
    base_link_point = self.tf.transformPoint('/base_link', NamedPoint.point)
    t = self.tf.getLatestCommonTime('/base_link', NamedPoint.header.frame_id)
    pos, quat = self.tf.lookupTransform('/base_link', NamedPoint.header.frame_id, t)
    height = pos[2]
    ground_point = base_link_point*np.abs(height)

    #self.points_pub.publish
    ground_named_point.point.x = ground_point[0]
    ground_named_point.point.y = ground_point[1]
    ground_named_point.point.z = ground_point[2]
    ground_named_point.header = NamedPoint.header
    ground_named_point.name = NamedPoint.name
    self.named_point_pub.publish(ground_named_point)


if __name__=="__main__":
  try:
    ray_to_points()
    rospy.spin()
  except rospy.ROSInterruptException: pass
