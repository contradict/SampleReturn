#!/usr/bin/env python

import threading
import collections
from copy import deepcopy
import numpy as np
import random

import rospy
import tf
import smach

import std_msgs.msg as std_msg
import geometry_msgs.msg as geometry_msg

from samplereturn_msgs.msg import SimpleMoveGoal
from samplereturn_msgs.msg import (VFHMoveGoal,
                                   VFHMoveAction,
                                   VFHMoveResult,
                                   VFHMoveFeedback)
import samplereturn.util as util
            
class BeaconReturn(smach.State):
    
    def __init__(self, label, tf_listener, beacon_enable, announcer):

        smach.State.__init__(self,
                             outcomes=['move',
                                       'spin',
                                       'mount',
                                       'preempted',
                                       'aborted'],
                             input_keys=['beacon_approach_pose',
                                         'beacon_observation_delay',
                                         'spin_velocity',
                                         'platform_point',
                                         'beacon_point',
                                         'stop_on_detection',
                                         'manager_dict',
                                         'odometry_frame',
                                         'platform_frame'],
                             output_keys=['move_target',
                                          'move_point_map',
                                          'simple_move',
                                          'report_beacon',
                                          'stop_on_detection',
                                          'report_sample',
                                          'active_manager',
                                          'beacon_point'])
        
        self.label = label
        self.tf_listener = tf_listener
        self.beacon_enable = beacon_enable
        self.announcer = announcer
        self.tried_spin = False

    def execute(self, userdata):
        #set the move manager key for the move mux
        userdata.active_manager = userdata.manager_dict[self.label]
        userdata.report_sample = False
        userdata.report_beacon = True
        self.beacon_enable.publish(True)        

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        #get our position in map
        current_pose = util.get_current_robot_pose(self.tf_listener,
                                                   userdata.platform_frame)        
        
        #hopeful distance to approach point
        distance_to_approach_point = util.point_distance_2d(current_pose.pose.position,
                                                            userdata.beacon_approach_pose.pose.position)

        #if we have been ignoring beacon detections prior to this,
        #we should clear them here, and wait for a fresh detection
        if not userdata.stop_on_detection:
            self.announcer.say("Wait ing for beacon.")
            userdata.beacon_point = None
            start_time = rospy.Time.now()
            while not rospy.core.is_shutdown_requested():
                rospy.sleep(0.5)
                if (userdata.beacon_point is not None) \
                or ((rospy.Time.now() - start_time) > userdata.beacon_observation_delay):
                    break
                
        if userdata.beacon_point is None: #beacon not in view
            #first hope, we can spin slowly and see the beacon
            if not self.tried_spin:
                self.announcer.say("Beacon not in view. Rotate ing")
                userdata.simple_move = SimpleMoveGoal(type=SimpleMoveGoal.SPIN,
                                                      angle = np.pi*2,
                                                      velocity = userdata.spin_velocity)        
                userdata.stop_on_detection = True
                self.tried_spin = True
                return 'spin'
            #already tried a spin, drive towards beacon_approach_point, stopping on detection
            elif distance_to_approach_point > 5.0:
                #we think we're far from approach_point, so try to go there
                self.announcer.say("Beacon not in view. Moving to approach point.")
                userdata.stop_on_detection = True
                self.tried_spin = False
                userdata.move_target = userdata.beacon_approach_pose
                return 'move'
            else:
                #we think we're looking at the beacon, but we don't see it, this is probably real bad
                self.announcer.say("Close to approach point in map.  Beacon not in view.  Search ing")
                search_pose = deepcopy(current_pose)                
                #try a random position nearby-ish, ignore facing
                search_pose.pose.position.x += random.randrange(-30, 30, 10) 
                search_pose.pose.position.y += random.randrange(-30, 30, 10)
                userdata.stop_on_detection = True
                self.tried_spin = False
                userdata.move_target = geometry_msg.PointStamped(search_pose.header,
                                                                 search_pose.pose.position)
                return 'move'               
                
        else: #beacon is in view
            current_yaw = util.get_current_robot_yaw(self.tf_listener,
                                                     userdata.platform_frame)
            yaw_to_platform = util.pointing_yaw(current_pose.pose.position,
                                                userdata.platform_point.point)
            yaw_error = util.unwind(yaw_to_platform - current_yaw)
            userdata.stop_on_detection = False
            self.tried_spin = False            
            #on the back side
            if current_pose.pose.position.x < 0:
                #try not to drive through the platform
                self.announcer.say("Back of beacon in view. Move ing to front")
                front_pose = deepcopy(userdata.beacon_approach_pose)
                front_pose.pose.position.y = 5.0 * np.sign(current_pose.pose.position.y)
                pointing_quat = util.pointing_quaternion_2d(front_pose.pose.position,
                                                            userdata.platform_point.point)
                front_pose.pose.orientation = pointing_quat
                userdata.move_target = front_pose
                return 'move'   
            elif distance_to_approach_point > 2.0:
                #on correct side of beacon, but far from approach point
                self.announcer.say("Beacon in view. Move ing to approach point")                
                userdata.move_target = userdata.beacon_approach_pose
                return 'move'   
            elif np.abs(yaw_error) > .2:
                #this means beacon is in view and we are within 1 meter of approach point
                #but not pointed so well at beacon
                self.announcer.say("At approach point. Rotate ing to beacon")
                userdata.simple_move = SimpleMoveGoal(type=SimpleMoveGoal.SPIN,
                                                      angle = yaw_error,
                                                      velocity = userdata.spin_velocity)                   
                return 'spin'
            else:    
                self.announcer.say("Measure ing beacon position.")
                userdata.stop_on_detection = False
                return 'mount'
        
        return 'aborted'
   
class CalculateMountMove(smach.State):
    
    def __init__(self, tf_listener, announcer):

        smach.State.__init__(self,
                             outcomes=['move',
                                       'aborted'],
                             input_keys=['platform_point',
                                         'odometry_frame',
                                         'beacon_mount_tolerance'],
                             output_keys=['simple_move'])

        self.tf_listener = tf_listener
        self.announcer = announcer

    def execute(self, userdata):
        #wait a sec for beacon pose to adjust the localization filter
        saved_point_odom = None
        #wait for a maximum of one minute, then mount anyway
        timeout_time = rospy.Time.now() + rospy.Duration(60.0)
        in_tolerance_count = 0
        while not rospy.core.is_shutdown_requested():
            rospy.sleep(3.0)    
            try:
                platform_point_odom = self.tf_listener.transformPoint(userdata.odometry_frame,
                                                                      userdata.platform_point)
            except tf.Exception:
                rospy.logwarn("LEVEL_TWO calculate_mount failed to transform platform point")
            if saved_point_odom is None:
                saved_point_odom = platform_point_odom
            else:
                correction_error = util.point_distance_2d(platform_point_odom.point,
                                                          saved_point_odom.point)
                saved_point_odom = platform_point_odom
                if correction_error < userdata.beacon_mount_tolerance:
                    in_tolerance_count += 1
                else:
                    in_tolerance_count = 0
                #if we get 3 in_tolerance corrections, mount
                if in_tolerance_count > 2:
                    break
                if rospy.Time.now() > timeout_time:
                    self.announcer.say("Beacon correction did not converge.")
                    break
                else:
                    self.announcer.say("Correction. {:.3f}".format(correction_error))
            
        yaw, distance = util.get_robot_strafe(self.tf_listener,
                                             userdata.platform_point)
        userdata.simple_move = SimpleMoveGoal(type=SimpleMoveGoal.STRAFE,
                                              angle = yaw,
                                              distance = distance,
                                              velocity = 0.5)        
        self.announcer.say("Initiate ing mount move.")
        return 'move'