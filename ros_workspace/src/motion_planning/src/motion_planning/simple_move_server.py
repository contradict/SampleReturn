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
from samplereturn_msgs.msg import (SimpleMoveGoal,
                                   SimpleMoveAction,
                                   SimpleMoveResult,
                                   SimpleMoveFeedback)
from motion_planning.simple_motion import SimpleMover


class SimpleMoveServer( object ):
    """
    SimpleMoveServer node
    """
    def __init__(self):
        self._goal_orientation_tolerance = \
                rospy.get_param("~goal_orientation_tolerance", 0.05)
        self.odometry_frame = rospy.get_param("~odometry_frame")
        self._mover = SimpleMover("~simple_motion_params/",
                                  stop_function = self.mover_stop_cb)

        self._as = actionlib.SimpleActionServer("simple_move", SimpleMoveAction,
                execute_cb = self.execute_cb, auto_start=False)

        self._as.start()

    def execute_cb(self, goal):
        """
        Called by SimpleActionServer when a new goal is ready
        """
        
        velocity = None if (goal.velocity == 0) else goal.velocity
        acceleration = None if (goal.acceleration == 0) else goal.acceleration        
 
 
        if goal.type == SimpleMoveGoal.SPIN:
            error = self._mover.execute_spin(goal.angle,
                                             max_velocity = velocity,
                                             acceleration = acceleration)
            rospy.loginfo("EXECUTED SPIN: %.1f, error %.3f" %( np.degrees(goal.angle),
                                                               np.degrees(error)))
        elif goal.type == SimpleMoveGoal.STRAFE:
            error = self._mover.execute_strafe(goal.angle,
                                               goal.distance,
                                               max_velocity = velocity,
                                               acceleration = acceleration)
            rospy.loginfo("EXECUTED STRAFE angle: %.1f, distance: %.1f, error %.3f" %(
                           np.degrees(goal.angle),
                           goal.distance,
                           error))
        else:
            rospy.logwarn('SIMPLE MOVE SERVER received invalid type')
            self._as.set_aborted(SimpleMoveResult(False, None))

        rospy.logdebug("Successfully completed goal.")
        self._as.set_succeeded(SimpleMoveResult(True, error))
                
 
    def mover_stop_cb(self):
        """
        Check to see if mover is running, if so, publish actionserver feedback.
        This also stops the mover if a preempt is requested on the action server.
        """
        
        #just publish 1 if we are running now
        self._as.publish_feedback(SimpleMoveFeedback(1))
        
        #if action server is preempted, stop the mover
        return self._as.is_preempt_requested()
