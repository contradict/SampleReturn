#!/usr/bin/env python
import roslib
import rospy
import numpy as np
import cv2
import copy

from geometry_msgs.msg import Vector3Stamped,Point,PointStamped
from sensor_msgs.msg import PointCloud2
from linemod_detector.msg import NamedPoint
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
from tf import TransformListener

class ray_to_points(object):
  # Alright, this thing is going to take in a ray (in the camera frame) from
  # a monocular camera or a stereo region lacking a disparity match. It's going
  # intersect that ray with a presumed flay ground plane and generate a point
  # cloud around that intersection point that incorporates some of our
  # geometrical uncertainty (most notably, pitch)

  def __init__(self):
    rospy.init_node('ray_to_points',log_level=rospy.DEBUG)

    self.named_point_sub = rospy.Subscriber('named_point', NamedPoint, self.handle_named_point)

    self.points_pub = rospy.Publisher('points', PointCloud2)
    self.named_point_pub = rospy.Publisher('point', NamedPoint)
    self.marker_pub = rospy.Publisher('marker', Marker)

    self.tf = TransformListener()

    self.pitch_error = rospy.get_param("~pitch_error",0.1)
    self.yaw_error = rospy.get_param("~yaw_error",0.1)

  def handle_named_point(self, point_in):
    rospy.logdebug("handle_named_point x: %s y: %s z: %s",
        point_in.point.x,
        point_in.point.y,
        point_in.point.z)
    point_stamped = PointStamped()
    point_stamped.header = point_in.header
    point_stamped.point = point_in.point

    ground_named_point, odom_named_point = self.cast_ray(point_stamped,self.tf,point_in.name)
    rospy.logdebug("ground_named_point %s",ground_named_point)
    self.named_point_pub.publish(ground_named_point)
    rospy.logdebug("odom_named_point %s",odom_named_point)
    self.send_marker(odom_named_point)

  def send_marker(self, named_pt):
    m=Marker()
    m.header = copy.deepcopy(named_pt.header)
    m.type=Marker.CYLINDER
    m.pose.position = named_pt.point
    m.pose.orientation.x=0.707
    m.pose.orientation.y=0.0
    m.pose.orientation.z=0.0
    m.pose.orientation.w=0.707
    m.scale.x=0.2
    m.scale.y=0.2
    m.scale.z=0.2
    m.color.r=0.8
    m.color.g=0.8
    m.color.b=0.8
    m.color.a=1.0
    #m.text=named_pt.name
    self.marker_pub.publish(m)

  def cast_ray(self, point_in, tf, name):
    base_link_point = tf.transformPoint('/base_link', point_in)
    t = tf.getLatestCommonTime('/base_link', point_in.header.frame_id)
    pos, quat = tf.lookupTransform('/base_link', point_in.header.frame_id, t)
    height = pos[2]

    x_slope = (base_link_point.point.x - pos[0])/(pos[2]-base_link_point.point.z)
    y_slope = (base_link_point.point.y - pos[1])/(pos[2]-base_link_point.point.z)

    ground_point = np.array([0.,0.,0.])
    ground_point[0] = x_slope*height
    ground_point[1] = y_slope*height

    ground_named_point = NamedPoint()
    ground_named_point.point.x = ground_point[0]
    ground_named_point.point.y = ground_point[1]
    ground_named_point.point.z = ground_point[2]
    ground_named_point.header = point_in.header
    ground_named_point.header.frame_id = 'base_link'
    ground_named_point.header.stamp = point_in.header.stamp
    ground_named_point.name = name

    odom_named_point = self.tf.transformPoint('/odom',ground_named_point)

    return ground_named_point, odom_named_point

  def make_point_cloud(Point):
    # Take a vector, nominally [x,y,1] and apply some rotation about x (pitch)
    # and about y (yaw) in the base_link frame. This will make a frustum that
    # can be sampled for a point cloud
    p = self.pitch_error
    pitch_mat = np.array([[1., 0., 0.],[0., np.cos(p), -np.sin(p)],[0., np.sin(p), np.cos(p)]])
    y = self.yaw_error
    yaw_mat = np.array([[np.cos(y), 0., np.sin(y)],[0., 1., 0.],[-np.sin(y), 0., np.cos(y)]])
    vec = np.array([0,0,0])
    vec[0] = Point.x
    vec[1] = Point.y
    vec[2] = Point.z
    down_left = np.dot(pitch_mat,np.dot(yaw_mat,vec))
    down_right = np.dot(pitch_mat,np.dot(-yaw_mat,vec))
    up_left = np.dot(-pitch_mat,np.dot(yaw_mat,vec))
    up_right = np.dot(-pitch_mat,np.dot(-yaw_mat,vec))



if __name__=="__main__":
  try:
    ray_to_points()
    rospy.spin()
  except rospy.ROSInterruptException: pass
