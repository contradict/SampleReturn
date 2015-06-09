#!/usr/bin/env python
import roslib
import rospy
import actionlib
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import tf
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Image
import actionlib_msgs.msg as action_msg
from nav_msgs.msg import Odometry
from sun_pointing.msg import ComputeAngleResult, ComputeAngleFeedback, ComputeAngleAction
from motion_planning import simple_motion
import samplereturn.util as util
import samplereturn_msgs.msg as samplereturn_msg
from samplereturn_msgs.msg import SimpleMoveGoal

class ComputeAngle(object):

  def __init__(self):

    node_params = util.get_node_params()
    self.odom_frame = node_params.odometry_frame
    self.tf_listener = tf.TransformListener()


    self.action_server = actionlib.SimpleActionServer('sun_pointing_action',
                                                      ComputeAngleAction,
                                                      self.run_compute_angle_action,
                                                      False)

    self.simple_mover = actionlib.SimpleActionClient("simple_move",
                                                     samplereturn_msg.SimpleMoveAction)
    
    self.spin_goal = SimpleMoveGoal(type=SimpleMoveGoal.SPIN,
                                    velocity = node_params.spin_velocity)  

    self.bridge = CvBridge()
    self.min_lightness = []
    self.img_angles = []

    self.turning = False
    self.starting_yaw = None
    self.current_yaw = None

    rospy.Timer(rospy.Duration(node_params.yaw_update_period),
                self.update_yaw)                

    rospy.Subscriber('image', Image, self.image_callback, None, 1)

    self.action_server.start()

  def update_yaw(self, msg):
    if self.turning:
      self.current_yaw = util.get_current_robot_yaw(self.tf_listener, self.odom_frame)
      self.action_server.publish_feedback(ComputeAngleFeedback(self.current_yaw))

  def image_callback(self, msg):
    if self.turning:
      img = np.asarray(self.bridge.imgmsg_to_cv2(msg,'bgr8'))
      lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
      self.min_lightness.append(np.min(lab[...,0]))
      self.img_angles.append(self.current_yaw)

  def run_compute_angle_action(self, goal):
    self.starting_yaw = util.get_current_robot_yaw(self.tf_listener, self.odom_frame)
    #spin all the way around
    self.turning = True
    self.spin_goal.angle = 2*np.pi
    if not self.execute_move_goal(self.spin_goal):
      self.turning = False
      return
    self.turning = False
    best_angle_index = np.argmax(self.min_lightness)
    best_angle = self.img_angles[best_angle_index]
    #return to best angle
    self.spin_goal.angle = util.unwind(best_angle-self.starting_yaw)
    if not self.execute_move_goal(self.spin_goal):
      return
    self.min_lightness = []
    self.img_angles = []
    #return the value of best angle in odometry_frame
    self.action_server.set_succeeded(ComputeAngleResult(best_angle))

  def execute_move_goal(self, goal):
    self.simple_mover.send_goal(goal)
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        move_state = self.simple_mover.get_state()
        if move_state not in util.actionlib_working_states:
            break
        if self.action_server.is_preempt_requested():
            self.action_server.set_preempted()
            self.simple_move.cancel_all_goals()
            return False
    
    if move_state == action_msg.GoalStatus.SUCCEEDED:
      return True
    else:
      rospy.logwarn("SIMPLE_MOVER failure in compute_sun_angle")
      self.action_server.set_aborted()
      return False
    
if __name__ == "__main__":
  rospy.init_node('sun_pointing')
  compute_angle = ComputeAngle()
  rospy.spin()
