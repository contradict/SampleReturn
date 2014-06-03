""" Simple Driving

Use simple_motion to drive to a position and yaw
"""
#!/usr/bin/env python
import numpy as np
import rospy
import actionlib
from tf.transformations import euler_from_quaternion
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Float64
from simple_driving.msg import (SimpleDrivingAction,
                                SimpleDrivingResult,
                                SimpleDrivingFeedback)
from samplereturn.simple_motion import SimpleMover

def euler_from_orientation(orientation):
    """
    euler angles from geometry_msgs/Quaterion
    """
    return euler_from_quaternion((
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
        ))

class SimpleDriving( object ):
    """
    SimpleDriving node
    """
    def __init__(self):
        self._goal_orientation_tolerance = \
                rospy.get_param("~goal_orientation_tolerance", 0.05)

        self._mover = SimpleMover("~")

        self._as = actionlib.SimpleActionServer("drive", SimpleDrivingAction,
                execute_cb = self.execute_cb, auto_start=False)

        self._robot_position = None
        self._odometry_sub = rospy.Subscriber("odometry", Odometry,
                self.odometry_cb)

        self._tf = tf.TransformListener()

        self._goal_odom = None

        self._as.start()

    def execute_cb(self, goal):
        """
        Called by SimpleActionServer when a new goal is ready
        """
        try:
            goal_local = self._tf.transformPose('base_link', goal.pose)
            goal_odom = self._tf.transformPose('odom', goal.pose)
        except tf.Exception, exc:
            self._as.set_aborted("Could not transform goal: %s"%exc)
            return
        self._goal_odom = goal_odom
        rospy.logdebug("Received goal, transformed to %s", goal_odom)

        # turn to face goal
        dyaw = np.arctan2( goal_local.pose.position.y,
                           goal_local.pose.position.x)
        rospy.logdebug("Rotating by %f.", dyaw)
        self._mover.execute_spin(dyaw,
                stop_function=self._as.is_preempt_requested)
        if self._as.is_preempt_requested():
            rospy.logdebug("Preempted during initial rotation.")
            self._as.set_preempted()
            return

        # drive to goal
        distance = np.hypot(goal_local.pose.position.x,
                goal_local.pose.position.y)
        rospy.logdebug("Moving %f meters ahead.", distance)
        self._mover.execute_strafe(0.0, distance,
                stop_function=self._as.is_preempt_requested)
        if self._as.is_preempt_requested():
            rospy.logdebug("Preempted during drive to goal.")
            self._as.set_preempted()
            return

        # turn to requested orientation
        dyaw = self.yaw_error()
        if np.abs(dyaw) > self._goal_orientation_tolerance:
            rospy.logdebug("Rotating to goal yaw %f.", dyaw)
            self._mover.execute_spin(dyaw,
                    stop_function=self._as.is_preempt_requested)
            if self._as.is_preempt_requested():
                rospy.logdebug("Preempted during final rotation.")
                self._as.set_preempted()
                return
        else:
            rospy.logdebug("Within tolerance of goal yaw, not rotating.")

        rospy.logdebug("Successfully completed goal.")
        self._as.set_succeeded(
                SimpleDrivingResult(True,
                self.position_error(),
                Float64(self.yaw_error())))

    def yaw_error(self):
        """
        Compute difference between present orientation and
        goal orientation.
        """
        return (euler_from_orientation(self._goal_odom.pose.orientation)[-1] -
            euler_from_orientation(self._robot_position.pose.orientation)[-1])

    def position_error(self):
        """
        Compute difference between present position and
        goal position.
        """
        return Point(
                self._goal_odom.pose.position.x -
                self._robot_position.pose.position.x,
                self._goal_odom.pose.position.x -
                self._robot_position.pose.position.x,
                0)


    def odometry_cb(self, odo):
        """
        Remeber current robot position
        """
        self._robot_position = PoseStamped(odo.header, odo.pose.pose)
        if self._as.is_active():
            self.publish_feedback()

    def publish_feedback(self):
        """
        Send actionserver feedback.
        """
        self._as.publish_feedback(
                SimpleDrivingFeedback( self.position_error(),
                    Float64(self.yaw_error())
                    )
                )

