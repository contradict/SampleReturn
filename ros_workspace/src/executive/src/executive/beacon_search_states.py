#!/usr/bin/env python

import threading
import collections
from copy import deepcopy
import numpy as np

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
            
class BeaconSearch(smach.State):
    
    def __init__(self, label, tf_listener, announcer):

        smach.State.__init__(self,
                             outcomes=['move',
                                       'spin',
                                       'mount',
                                       'aborted'],
                             input_keys=['beacon_approach_pose',
                                         'move_velocity',
                                         'spin_velocity',
                                         'platform_point',
                                         'beacon_point',
                                         'stop_on_beacon',
                                         'vfh_result',
                                         'manager_dict',
                                         'world_fixed_frame',
                                         'odometry_frame'],
                             output_keys=['move_target',
                                          'move_point_map',
                                          'simple_move',
                                          'stop_on_beacon',
                                          'active_manager',
                                          'beacon_point',
                                          'outbound'])
        
        self.label = label
        self.tf_listener = tf_listener
        self.announcer = announcer
        self.tried_spin = False

    def execute(self, userdata):
        #set the move manager key for the move mux
        userdata.active_manager = userdata.manager_dict[self.label]
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        #ignore samples after this (applies to mount moves)
        #clear previous move_point_map
        userdata.move_point_map = None
        map_header = std_msg.Header(0, rospy.Time(0), userdata.world_fixed_frame)
       
        #get our position in map
        current_pose = util.get_current_robot_pose(self.tf_listener,
                                                   userdata.world_fixed_frame)        
        
        #this is possibly a highly inaccurate number.   If we get to the approach point,
        #and don't see the beacon, the map is probably messed up
        distance_to_approach_point = util.point_distance_2d(current_pose.pose.position,
                                                            userdata.beacon_approach_pose.pose.position)

        rotate_to_clear = False
        if userdata.vfh_result == VFHMoveResult.STARTED_BLOCKED:
            rospy.loginfo("LEVEL_TWO beacon search setting rotate_to_clear = True")
            rotate_to_clear = True

        #if we have been ignoring beacon detections prior to this,
        #we should clear them here, and wait for a fresh detection
        if not userdata.stop_on_beacon:
            userdata.beacon_point = None
            rospy.sleep(4.0)
        
        if userdata.beacon_point is None: #beacon not in view
            #first hope, we can spin slowly and see the beacon
            if not self.tried_spin:
                self.announcer.say("Beacon not in view. Rotate ing")
                userdata.simple_move = SimpleMoveGoal(type=SimpleMoveGoal.SPIN,
                                                      angle = np.pi*2,
                                                      velocity = userdata.spin_velocity)        
                userdata.stop_on_beacon = True
                self.tried_spin = True
                return 'spin'
            #already tried a spin, drive towards beacon_approach_point, stopping on detection
            elif distance_to_approach_point > 5.0:
                #we think we're far from approach_point, so try to go there
                self.announcer.say("Beacon not in view. Search ing")
                userdata.stop_on_beacon = True
                self.tried_spin = False
                #set move_point_map to enable localization correction
                userdata.move_target = userdata.beacon_approach_pose
                return 'move'
            else:
                #we think we're near the beacon, but we don't see it
                #right now, that is just to move 30 m on other side of the beacon that we don't know about
                self.announcer.say("Close to approach point in map.  Beacon not in view.  Search ing")
                search_pose = deepcopy(userdata.beacon_approach_pose)                
                #invert the approach_point, and try again
                search_pose.pose.position.x *= -1
                userdata.stop_on_beacon = True
                self.tried_spin = False
                userdata.move_target = search_pose
                return 'move'               
                
        else: #beacon is in view
            current_yaw = util.get_current_robot_yaw(self.tf_listener,
                                                     userdata.world_fixed_frame)
            yaw_to_platform = util.pointing_yaw(current_pose.pose.position,
                                                userdata.platform_point.point)
            yaw_error = util.unwind(yaw_to_platform - current_yaw)
            userdata.stop_on_beacon = False
            self.tried_spin = False            
            #on the back side
            if current_pose.pose.position.x < 0:
                #try not to drive through the platform
                self.announcer.say("Back of beacon in view. Move ing to front")
                front_pose = deepcopy(userdata.beacon_approach_pose)
                front_pose.pose.position.y = 5.0 * np.sign(current_pose.pose.position.y)
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
                userdata.stop_on_beacon = False
                return 'mount'
        
        return 'aborted'
   
class MountManager(smach.State):
    
    def __init__(self, tf_listener, announcer):

        smach.State.__init__(self,
                             outcomes=['move',
                                       'final',
                                       'aborted'],
                             input_keys=['platform_point',
                                         'beacon_point',
                                         'beacon_mount_step'],
                             output_keys=['simple_move',
                                          'stop_on_sample',
                                          'beacon_point'])

        self.tf_listener = tf_listener
        self.announcer = announcer

    def execute(self, userdata):
        #disable obstacle checking!
        rospy.sleep(10.0) #wait a sec for beacon pose to catch up
        target_point = deepcopy(userdata.platform_point)
        target_point.header.stamp = rospy.Time(0)
        yaw, distance = util.get_robot_strafe(self.tf_listener,
                                             target_point)
        move = SimpleMoveGoal(type=SimpleMoveGoal.STRAFE,
                              angle = yaw,
                              distance = distance,
                              velocity = 0.5)        
        userdata.simple_move = move
        userdata.stop_on_sample = False
        self.announcer.say("Initiate ing mount move.")
        return 'final'