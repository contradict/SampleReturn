#!/usr/bin/env python
import roslib
import rospy
import actionlib
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from sun_pointing.msg import ComputeAngleResult, ComputeAngleFeedback, ComputeAngleAction
from samplereturn import simple_motion

class ComputeAngle(object):

  def __init__(self):
    rospy.Subscriber('image', Image, self.image_callback, None, 1)
    rospy.Subscriber('odometry', Odometry, self.odometry_callback, None, 1)

    self.action_server = actionlib.SimpleActionServer('sun_pointing_action', ComputeAngleAction, self.run_compute_angle_action, False)

    self.bridge = CvBridge()
    self.max_indices = []
    self.img_angles = []

    self.turning = False
    self.starting_yaw = None

    self.mover = simple_motion.SimpleMotion('~compute_sun_angle_params/')

    self.action_server.start()

  def odometry_callback(self, msg):
    if self.starting_yaw == None:
      self.starting_yaw = euler_from_quaternion(msg.pose.pose.quaternion)[-1]

    self.current_yaw = euler_from_quaternion(msg.pose.pose.quaternion)[-1]

    if self.turning:
      self.action_server.set_feedback(ComputeAngleFeedback(self.current_yaw))

  def image_callback(self, msg):
    if self.turning:
      img = np.asarray(self.bridge.imgmsg_to_cv(msg,'bgr8'))
      gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
      hist = cv2.calcHist([gray],[0],None,[256],[0,256])
      max_index = np.argmax(hist)
      self.max_indices.append(max_index)
      self.img_angles.append(self.current_yaw)

  def run_compute_angle_action(self, goal):
    self.starting_yaw = None
    # First half of spin
    self.turning = True
    self.mover.execute_spin(np.pi)
    if self.action_server.is_preempt_requested():
      return
    # Second half of spin
    self.mover.execute_spin(np.pi)
    if self.action_server.is_preempt_requested():
      return
    self.turning = False
    best_angle_index = np.argmax(self.max_indices)
    best_angle = self.img_angles[best_angle_index]
    # Return best angle
    self.mover.execute_spin(best_angle-self.starting_yaw)
    self.action_server.set_succeeded(ComputeAngleResult(best_angle))


if __name__ == "__main__":
  rospy.init_node('sun_pointing')
  compute_angle = ComputeAngle()
  rospy.spin()
