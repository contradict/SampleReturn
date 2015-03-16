#!/usr/bin/env python
import math
import threading
import random
from copy import deepcopy
from collections import deque
import numpy as np

import smach
import smach_ros
import rospy
import actionlib
import tf

import std_msgs.msg as std_msg
import actionlib_msgs.msg as action_msg
import nav_msgs.msg as nav_msg
import visual_servo_msgs.msg as visual_servo_msg
import samplereturn_msgs.msg as samplereturn_msg
import move_base_msgs.msg as move_base_msg
import geometry_msgs.msg as geometry_msg
import platform_motion_msgs.msg as platform_msg
import platform_motion_msgs.srv as platform_srv
import visualization_msgs.msg as vis_msg

import motion_planning.simple_motion as simple_motion

from executive.executive_states import SelectMotionMode
from executive.executive_states import AnnounceState
from executive.executive_states import ExecuteSimpleMove
from executive.executive_states import ExecuteVFHMove
from executive.executive_states import WaitForFlagState

import samplereturn.util as util
import samplereturn.bresenham as bresenham

from samplereturn_msgs.msg import (SimpleMoveGoal,
                                   SimpleMoveAction,
                                   SimpleMoveResult,
                                   SimpleMoveFeedback)

class LevelTwoStar(object):
    
    def __init__(self):
        
        rospy.on_shutdown(self.shutdown_cb)
        node_params = util.get_node_params()
        self.tf_listener = tf.TransformListener()

        self.world_fixed_frame = rospy.get_param("world_fixed_frame", "map")
        self.odometry_frame = rospy.get_param("odometry_frame", "odom")
        
        #make a Point msg out of beacon_approach_point, and a pose, at that point, facing beacon
        header = std_msg.Header(0, rospy.Time(0), self.odometry_frame)
        point = geometry_msg.Point(node_params.beacon_approach_point['x'],
                                   node_params.beacon_approach_point['y'], 0)
        beacon_facing_orientation = geometry_msg.Quaternion(*tf.transformations.quaternion_from_euler(0,0,math.pi))
        pose = geometry_msg.Pose(position = point, orientation = beacon_facing_orientation)
        self.beacon_approach_pose = geometry_msg.PoseStamped(header = header, pose = pose)
        
        #also need platform point
        platform_point = geometry_msg.Point( 0, 0, 0)
        self.platform_point = geometry_msg.PointStamped(header, platform_point)
        
        #interfaces
        self.announcer = util.AnnouncerInterface("audio_search")
        self.CAN_interface = util.CANInterface()
       
        #movement action servers        
        self.vfh_mover = actionlib.SimpleActionClient("vfh_move",
                                                       samplereturn_msg.VFHMoveAction)

        self.simple_mover = actionlib.SimpleActionClient("simple_move",
                                                       samplereturn_msg.SimpleMoveAction)

        #get the star shape
        self.spokes = self.get_hollow_star(node_params.spoke_count,
                                           node_params.spoke_length,
                                           node_params.first_spoke_offset,
                                           node_params.star_hub_radius,
                                           node_params.spin_step)
                       
        self.state_machine = smach.StateMachine(
                outcomes=['complete', 'preempted', 'aborted'],
                input_keys = ['action_goal'],
                output_keys = ['action_result'])
    
        self.state_machine.userdata.spokes = self.spokes
        self.state_machine.userdata.star_hub_radius = node_params.star_hub_radius

        #these are important values! master frame id and return timing
        self.state_machine.userdata.world_fixed_frame = self.world_fixed_frame
        self.state_machine.userdata.odometry_frame = self.odometry_frame
        self.state_machine.userdata.start_time = rospy.Time.now()
        self.state_machine.userdata.return_time = rospy.Time.now() + \
                                                  rospy.Duration(node_params.return_time_minutes*60)
        self.state_machine.userdata.pre_cached_id = samplereturn_msg.NamedPoint.PRE_CACHED
        
        #dismount move
        self.state_machine.userdata.dismount_move = node_params.dismount_move

        #beacon approach
        self.state_machine.userdata.beacon_approach_pose = self.beacon_approach_pose
        self.state_machine.userdata.beacon_mount_step = node_params.beacon_mount_step
        self.state_machine.userdata.platform_point = self.platform_point
        
        #search line parameters
        self.state_machine.userdata.move_velocity = node_params.move_velocity
        self.state_machine.userdata.spin_velocity = node_params.spin_velocity
        self.state_machine.userdata.min_spin_radius = node_params.min_spin_radius
        self.state_machine.userdata.last_spin_radius = 0
        self.state_machine.userdata.spin_step = node_params.spin_step
        self.state_machine.userdata.line_yaw = None #IN RADIANS!
        self.state_machine.userdata.outbound = False
        
        #stop function flags
        self.state_machine.userdata.stop_on_sample = False
        self.state_machine.userdata.stop_on_beacon = False

        #subscriber controlled userdata
        self.state_machine.userdata.paused = False        
        self.state_machine.userdata.detected_sample = None
        self.state_machine.userdata.beacon_point = None
        
        #use these as booleans in remaps
        self.state_machine.userdata.true = True
        self.state_machine.userdata.false = False
    
        #motion mode stuff
        planner_mode = node_params.planner_mode
        MODE_PLANNER = getattr(platform_srv.SelectMotionModeRequest, planner_mode)
        MODE_SERVO = platform_srv.SelectMotionModeRequest.MODE_SERVO
        MODE_PAUSE = platform_srv.SelectMotionModeRequest.MODE_PAUSE
    
        with self.state_machine:
            
            smach.StateMachine.add('START_LEVEL_TWO',
                                   StartLeveLTwo(input_keys=['dismount_move'],
                                                 output_keys=['action_result',
                                                              'simple_move'],
                                                 outcomes=['next']),
                                   transitions = {'next':'ANNOUNCE_LEVEL_TWO'})
            
            smach.StateMachine.add('ANNOUNCE_LEVEL_TWO',
                                   AnnounceState(self.announcer,
                                                 'Enter ing level two mode'),
                                   transitions = {'next':'SELECT_PLANNER'})
            
            smach.StateMachine.add('SELECT_PLANNER',
                                    SelectMotionMode(self.CAN_interface,
                                                     MODE_PLANNER),
                                    transitions = {'next':'DISMOUNT_MOVE',
                                                   'paused':'WAIT_FOR_UNPAUSE',
                                                  'failed':'LEVEL_TWO_ABORTED'})
            
            smach.StateMachine.add('WAIT_FOR_UNPAUSE',
                                   WaitForFlagState('paused',
                                                    flag_trigger_value = False,
                                                    timeout = 15,
                                                    announcer = self.announcer,
                                                    start_message ='System is paused. Un pause to continue level two'),
                                   transitions = {'next':'SELECT_PLANNER',
                                                  'timeout':'WAIT_FOR_UNPAUSE',
                                                  'preempted':'LEVEL_TWO_PREEMPTED'})
            
            smach.StateMachine.add('DISMOUNT_MOVE',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'STAR_MANAGER',
                                                  'sample_detected':'STAR_MANAGER',
                                                  'preempted':'LEVEL_TWO_PREEMPTED',
                                                  'aborted':'LEVEL_TWO_ABORTED'},
                                   remapping = {'stop_on_sample':'false'})
            
            smach.StateMachine.add('STAR_MANAGER',
                                   StarManager(self.tf_listener,
                                               self.announcer),
                                   transitions = {'next_spoke':'LINE_MANAGER',
                                                  'return_home':'ANNOUNCE_RETURN_HOME'})

            smach.StateMachine.add('LINE_MANAGER',
                                   SearchLineManager(self.tf_listener),
                                   transitions = {'move':'LINE_MOVE',
                                                  'line_end':'STAR_MANAGER',
                                                  'return_home':'ANNOUNCE_RETURN_HOME',
                                                  'preempted':'LEVEL_TWO_PREEMPTED',
                                                  'aborted':'LEVEL_TWO_ABORTED'})

            smach.StateMachine.add('LINE_MOVE',
                                   ExecuteVFHMove(self.vfh_mover),
                                   transitions = {'complete':'SPIN_MANAGER',
                                                  'missed_target':'ANNOUNCE_MISSED_TARGET',
                                                  'blocked':'ANNOUNCE_LINE_BLOCKED',
                                                  'off_course':'ANNOUNCE_OFF_COURSE',
                                                  'sample_detected':'PURSUE_SAMPLE',
                                                  'preempted':'LEVEL_TWO_PREEMPTED',
                                                  'aborted':'LEVEL_TWO_ABORTED'},
                                   remapping = {'pursue_samples':'true'})
            
            smach.StateMachine.add('ANNOUNCE_LINE_BLOCKED',
                                   AnnounceState(self.announcer, 'Line Blocked.'),
                                   transitions = {'next':'STAR_MANAGER'})
            
            smach.StateMachine.add('ANNOUNCE_OFF_COURSE',
                                   AnnounceState(self.announcer, 'Off Course.'),
                                   transitions = {'next':'STAR_MANAGER'})
            
            smach.StateMachine.add('ANNOUNCE_MISSED_TARGET',
                                   AnnounceState(self.announcer, 'Missed Target'),
                                   transitions = {'next':'SPIN_MANAGER'})
            
            smach.StateMachine.add('SPIN_MANAGER',
                                   SpinManager(self.tf_listener),
                                   transitions = {'spin':'ROTATE',
                                                  'move':'LINE_MANAGER',
                                                  'preempted':'LEVEL_TWO_PREEMPTED',
                                                  'aborted':'LEVEL_TWO_ABORTED'})
            
            smach.StateMachine.add('ROTATE',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'LINE_MANAGER',
                                                  'sample_detected':'PURSUE_SAMPLE',
                                                  'preempted':'LEVEL_TWO_PREEMPTED',
                                                  'aborted':'LEVEL_TWO_ABORTED'},
                                   remapping = {'stop_on_sample':'true'})
            
            @smach.cb_interface(input_keys=['detected_sample'])
            def pursuit_goal_cb(userdata, request):
                goal = samplereturn_msg.GeneralExecutiveGoal()
                goal.input_point = userdata.detected_sample
                goal.input_string = "level_two_pursuit_request"
                return goal
            
            @smach.cb_interface(input_keys=['line_points','next_line_point'],
                                output_keys=['line_points','detected_sample'])
            def pursuit_result_cb(userdata, status, result):
                #clear samples after a pursue action
                userdata.detected_sample = None
                #reload the next spoke point, line manager will pop it back
                #off the array and we will continue to go there
                userdata.line_points.appendleft(userdata.next_line_point)
                            
            smach.StateMachine.add('PURSUE_SAMPLE',
                                  smach_ros.SimpleActionState('pursue_sample',
                                  samplereturn_msg.GeneralExecutiveAction,
                                  goal_cb = pursuit_goal_cb,
                                  result_cb = pursuit_result_cb),
                                  transitions = {'succeeded':'ANNOUNCE_RETURN_TO_SEARCH',
                                                 'aborted':'ANNOUNCE_RETURN_TO_SEARCH'})

            smach.StateMachine.add('ANNOUNCE_RETURN_TO_SEARCH',
                                   AnnounceState(self.announcer,
                                                 'Return ing to search line'),
                                   transitions = {'next':'LINE_MANAGER'})   

            smach.StateMachine.add('ANNOUNCE_RETURN_HOME',
                                   AnnounceState(self.announcer,
                                                 'Return ing to home platform'),
                                   transitions = {'next':'BEACON_SEARCH'})
 
            smach.StateMachine.add('BEACON_SEARCH',
                                   BeaconSearch(self.tf_listener, self.announcer),
                                   transitions = {'mount':'MOUNT_MANAGER',
                                                  'move':'BEACON_SEARCH_MOVE',
                                                  'spin':'BEACON_SEARCH_SPIN',
                                                  'aborted':'LEVEL_TWO_ABORTED'})
 
            smach.StateMachine.add('BEACON_SEARCH_SPIN',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'BEACON_SEARCH',
                                                  'sample_detected':'BEACON_SEARCH',
                                                  'preempted':'LEVEL_TWO_PREEMPTED',
                                                  'aborted':'LEVEL_TWO_ABORTED'})
            
            #return to start along the approach point
            #if the path is ever blocked just give up and return to the level_two search
            smach.StateMachine.add('BEACON_SEARCH_MOVE',
                                   ExecuteVFHMove(self.vfh_mover),
                                   transitions = {'complete':'BEACON_SEARCH',
                                                  'blocked':'BEACON_SEARCH',
                                                  'missed_target':'BEACON_SEARCH',
                                                  'off_course':'BEACON_SEARCH',
                                                  'sample_detected':'BEACON_SEARCH',
                                                  'preempted':'BEACON_SEARCH',
                                                  'aborted':'LEVEL_TWO_ABORTED'},
                                   remapping = {'pursue_samples':'true',
                                                'detected_sample':'beacon_point'})
            
 
            smach.StateMachine.add('MOUNT_MANAGER',
                                   MountManager(self.tf_listener, self.announcer),
                                   transitions = {'move':'MOUNT_MOVE',
                                                  'final':'MOUNT_FINAL',
                                                  'aborted':'LEVEL_TWO_ABORTED'})
 
            smach.StateMachine.add('MOUNT_MOVE',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'MOUNT_MANAGER',
                                                  'sample_detected':'MOUNT_MANAGER',
                                                  'preempted':'LEVEL_TWO_PREEMPTED',
                                                  'aborted':'LEVEL_TWO_ABORTED'})
 
            smach.StateMachine.add('MOUNT_FINAL',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'DESELECT_PLANNER',
                                                  'sample_detected':'MOUNT_MANAGER',                                                  
                                                  'preempted':'LEVEL_TWO_PREEMPTED',
                                                  'aborted':'LEVEL_TWO_ABORTED'})

            smach.StateMachine.add('DESELECT_PLANNER',
                                    SelectMotionMode(self.CAN_interface,
                                                     MODE_PAUSE),
                                    transitions = {'next':'complete',
                                                   'paused':'LEVEL_TWO_ABORTED',
                                                   'failed':'LEVEL_TWO_ABORTED'})   
    
            smach.StateMachine.add('LEVEL_TWO_PREEMPTED',
                                  LevelTwoPreempted(),
                                   transitions = {'next':'preempted'})
            
            smach.StateMachine.add('LEVEL_TWO_ABORTED',
                                   LevelTwoAborted(),
                                   transitions = {'next':'aborted'})
            
        #action server wrapper    
        level_two_server = smach_ros.ActionServerWrapper(
            'level_two', samplereturn_msg.GeneralExecutiveAction,
            wrapped_container = self.state_machine,
            succeeded_outcomes = ['complete'],
            preempted_outcomes = ['preempted'],
            aborted_outcomes = ['aborted'],
            goal_key = 'action_goal',
            result_key = 'action_result')
        
        #introspection server
        sls = smach_ros.IntrospectionServer('smach_grab_introspection',
                                            self.state_machine,
                                            '/START_LEVEL_TWO')

        rospy.Subscriber('detected_sample_search',
                        samplereturn_msg.NamedPoint,
                        self.sample_update)
        
        rospy.Subscriber("beacon_pose",
                        geometry_msg.PoseWithCovarianceStamped,
                        self.beacon_update)
        
        rospy.Subscriber("pause_state", std_msg.Bool, self.pause_state_update)
        
        #start action servers and services
        sls.start()
        level_two_server.run_server()
        rospy.spin()
        sls.stop()
    
    def pause_state_update(self, msg):
        self.state_machine.userdata.paused = msg.data
        
    def sample_update(self, sample):
        try:
            self.tf_listener.waitForTransform(self.odometry_frame,
                                              sample.header.frame_id,
                                              sample.header.stamp,
                                              rospy.Duration(1.0))
            point_in_frame = self.tf_listener.transformPoint(self.odometry_frame, sample)
            sample.point = point_in_frame.point
            self.state_machine.userdata.detected_sample = sample
        except tf.Exception:
            rospy.logwarn("LEVEL_TWO failed to transform search detection point %s->%s",
                          sample.header.frame_id, self.odometry_frame)
            
    def beacon_update(self, beacon_pose):
        beacon_point = geometry_msg.PointStamped(beacon_pose.header,
                                                 beacon_pose.pose.pose.position)
        self.state_machine.userdata.beacon_point = beacon_point

    def get_hollow_star(self, spoke_count, spoke_length, offset, hub_radius, spin_step):
        #This creates a list of dictionaries.  The yaw entry is a useful piece of information
        #for some of the state machine states.  The other entry is the list of points that
        #the robot should try to achieve along the spoke definied by the yaw
        
        offset = np.radians(offset)
        yaws = list(np.linspace(0 + offset, -2*np.pi + offset, spoke_count, endpoint=False))
        spokes = []
        for yaw in yaws:
            yaw = util.unwind(yaw)
            radius = hub_radius
            points = deque([])
            while radius < spoke_length:
                header = std_msg.Header(0, rospy.Time(0), self.world_fixed_frame)
                pos = geometry_msg.Point(radius*math.cos(yaw),
                                         radius*math.sin(yaw), 0)
                point = geometry_msg.PointStamped(header, pos)
                radius += spin_step
                if radius > spoke_length: radius = spoke_length
                points.append(point)
            spokes.append({'yaw':yaw,
                           'points':points})
        
        return deque(spokes)
 
    def shutdown_cb(self):
        self.state_machine.request_preempt()
        while self.state_machine.is_running():
            rospy.sleep(0.1)
        rospy.logwarn("EXECUTIVE LEVEL_TWO STATE MACHINE EXIT")
        rospy.sleep(0.2) #hideous hack delay to let action server get its final message out
     
#searches the globe   
class StartLeveLTwo(smach.State):

    def execute(self, userdata):
        
        result = samplereturn_msg.GeneralExecutiveResult()
        result.result_string = 'initialized'
        userdata.action_result = result
        
        #create the dismount_move
        move = SimpleMoveGoal(type=SimpleMoveGoal.STRAFE,
                              **userdata.dismount_move)

        userdata.simple_move = move

        return 'next'

class StarManager(smach.State):
    def __init__(self, tf_listener, announcer):
        smach.State.__init__(self,
                             input_keys = ['line_yaw',
                                           'outbound',
                                           'spokes',
                                           'world_fixed_frame'],
                             output_keys = ['line_yaw',
                                            'line_points',
                                            'outbound',
                                            'spokes',
                                            'stop_on_sample'],
                             outcomes=['next_spoke',
                                       'return_home',
                                       'preempted', 'aborted'])
        
        self.tf_listener = tf_listener
        self.announcer = announcer  

    def execute(self, userdata):

        userdata.stop_on_sample = True

        if len(userdata.spokes) == 0:
            rospy.loginfo("STAR MANAGER finished last spoke")
            return 'return_home'

        if not userdata.outbound:
            #just finished inbound spoke, turn around and head outwards again
            spoke = userdata.spokes.popleft()
            userdata.line_yaw = spoke['yaw']
            userdata.line_points = deque(spoke['points'])
            userdata.outbound = True
            rospy.loginfo("STAR_MANAGER starting spoke: %.2f" %(userdata.line_yaw))
            self.announcer.say("Start ing on spoke, Yaw %s" % (int(math.degrees(userdata.line_yaw))))
        else:
            #just finished outbound spoke, head towards first point in next spoke
            userdata.outbound = False
            userdata.line_points = deque([userdata.spokes[0]['points'].popleft()])
            rospy.loginfo("STAR_MANAGER returning to hub point: %s" %(userdata.spokes[0]['points'][0]))
            self.announcer.say("Return ing on spoke, Yaw " + str(int(math.degrees(userdata.line_yaw))))

        return 'next_spoke'

class SpinManager(smach.State):

    def __init__(self, tf_listener):
        smach.State.__init__(self,
                             outcomes=['spin', 'move', 'preempted', 'aborted'],
                             input_keys=['line_points',
                                         'min_spin_radius',
                                         'spin_velocity',
                                         'outbound',
                                         'world_fixed_frame',],
                             output_keys=['simple_move',
                                          'beacon_point',
                                          'distance_to_hub',
                                          'last_align_time',])
  
        self.tf_listener = tf_listener
    
    def execute(self, userdata):
        #if heading out, and further than spin step, and not on last move, spin
        if userdata.outbound:
            distance = util.get_robot_distance_to_origin(self.tf_listener, userdata.world_fixed_frame)
            if (len(userdata.line_points) > 0) and (distance > userdata.min_spin_radius):     
                userdata.simple_move = SimpleMoveGoal(type=SimpleMoveGoal.SPIN,
                                                      angle = 2.*math.pi,
                                                      velocity = userdata.spin_velocity)        
                return 'spin'                

        #otherwise, on to next movement        
        return 'move'    

#drive to detected sample location        
class SearchLineManager(smach.State):
    def __init__(self, tf_listener):
        smach.State.__init__(self,
                             input_keys = ['line_points',
                                           'outbound',
                                           'return_time',
                                           'detected_sample',
                                           'world_fixed_frame',
                                           'odometry_frame',
                                           'move_velocity',
                                           'spin_velocity',
                                           'beacon_point',
                                           'last_align_time'],
                             output_keys = ['move_goal',
                                            'line_points',
                                            'next_line_point',
                                            'beacon_point',
                                            'last_align_time'],
                             outcomes=['move',
                                       'line_end',
                                       'return_home',
                                       'preempted', 'aborted'])
        
        self.tf_listener = tf_listener

    def execute(self, userdata):

        odometry_frame = userdata.odometry_frame
        
        if rospy.Time.now() > userdata.return_time:
            return 'return_home'
        
        #if len of points is zero we have reached the start or end of a line
        if len(userdata.line_points) == 0:
            return 'line_end'
        else:
            target_point = userdata.line_points.popleft()
            #store the next point in case of interruption by pursuit.
            userdata.next_line_point = target_point

        try:
            self.tf_listener.waitForTransform(odometry_frame,
                                              target_point.header.frame_id,
                                              target_point.header.stamp,
                                              rospy.Duration(1.0))
            point_in_odom = self.tf_listener.transformPoint(odometry_frame, target_point)
        except tf.Exception:
            rospy.logwarn("LEVEL_TWO failed to transform line manager point %s->%s",
                          target_point.header.frame_id, odometry_frame)
            return 'aborted'

        header = std_msg.Header(0, rospy.Time(0), odometry_frame)
        pose = geometry_msg.Pose(position = point_in_odom.point,
                                 orientation = geometry_msg.Quaternion())
        target_pose = geometry_msg.PoseStamped(header, pose)
        goal = samplereturn_msg.VFHMoveGoal(target_pose = target_pose,
                                            move_velocity = userdata.move_velocity,
                                            spin_velocity = userdata.spin_velocity,
                                            orient_at_target = False)
        userdata.move_goal = goal
        return 'move'
            
class BeaconSearch(smach.State):
    
    def __init__(self, tf_listener, announcer):

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
                                         'world_fixed_frame'],
                             output_keys=['move_goal',
                                          'simple_move',
                                          'stop_on_sample',
                                          'stop_on_beacon',
                                          'beacon_point',
                                          'outbound'])
        
        self.tf_listener = tf_listener
        self.announcer = announcer
        self.tried_spin = False

    def execute(self, userdata):
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        #ignore samples now
        userdata.stop_on_sample = False
        
        current_pose = util.get_current_robot_pose(self.tf_listener,
                                                   userdata.world_fixed_frame)        
        
        #this is possibly a highly inaccurate number.   If we get to the approach point,
        #and don't see the beacon, the map is probably messed up
        distance_to_approach_point = util.point_distance_2d(current_pose.pose.position,
                                                            userdata.beacon_approach_pose.pose.position)

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
                                                      angle = math.pi*2,
                                                      velocity = userdata.spin_velocity)        
                userdata.stop_on_beacon = True
                self.tried_spin = True
                return 'spin'
            #already tried a spin, drive towards beacon_approach_point, stopping on detection
            elif distance_to_approach_point > 5.0:
                #we think we're far from approach_point, so try to go there
                self.announcer.say("Beacon not in view. Search ing")
                goal = samplereturn_msg.VFHMoveGoal(target_pose = userdata.beacon_approach_pose,
                                                    move_velocity = userdata.move_velocity,
                                                    spin_velocity = userdata.spin_velocity)
                userdata.move_goal = goal                
                userdata.stop_on_beacon = True
                self.tried_spin = False
                return 'move'
            else:
                #gotta add some heroic shit here
                #we think we're near the beacon, but we don't see it
                #right now, that is just to move 30 m on other side of the beacon that we don't know about
                self.announcer.say("Close to approach point in map.  Beacon not in view.  Search ing")
                search_pose = deepcopy(userdata.beacon_approach_pose)                
                #invert the approach_point, and try again
                search_pose.pose.position.x *= -1
                goal = samplereturn_msg.VFHMoveGoal(target_pose = userdata.search_pose,
                                                    move_velocity = userdata.move_velocity,
                                                    spin_velocity = userdata.spin_velocity)
                userdata.move_goal = goal                        
                userdata.stop_on_beacon = True
                self.tried_spin = False
                return 'move'               
                
        else: #beacon is in view
            #need to add some stuff here to get to other side of beacon if viewing back
            current_yaw = util.get_current_robot_yaw(self.tf_listener,
                                                       userdata.world_fixed_frame)
            yaw_to_platform = util.pointing_yaw(current_pose.pose.position,
                                                userdata.platform_point.point)
            yaw_error = util.unwind(yaw_to_platform - current_yaw)
            userdata.stop_on_beacon = False
            self.tried_spin = False            
            #on the back side
            if current_pose.pose.position.x < 0:
                front_pose = deepcopy(userdata.beacon_approach_pose)
                #try not to drive through the platform
                front_pose.pose.position.y = 5.0 * np.sign(current_pose.pose.position.y)
                goal = samplereturn_msg.VFHMoveGoal(target_pose = userdata.front_pose,
                                                    move_velocity = userdata.move_velocity,
                                                    spin_velocity = userdata.spin_velocity)
                userdata.move_goal = goal                          
                self.announcer.say("Back of beacon in view. Move ing to front")
                return 'move'   
            elif distance_to_approach_point > 2.0:
                #on correct side of beacon, but far from approach point
                goal = samplereturn_msg.VFHMoveGoal(target_pose = userdata.beacon_approach_pose,
                                                    move_velocity = userdata.move_velocity,
                                                    spin_velocity = userdata.spin_velocity)
                userdata.move_goal = goal       
                self.announcer.say("Beacon in view. Move ing to approach point")                
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
                self.announcer.say("Initiate ing platform mount")
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
        self.announcer.say("Execute ing final mount move")
        return 'final'
    
class LevelTwoPreempted(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next'])
        
    def execute(self, userdata):
        
        return 'next'

class LevelTwoAborted(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next'])
        
    def execute(self, userdata):
        
        return 'next'
    

    
    
    
