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

from samplereturn_msgs.msg import (SimpleMoveGoal,
                                   SimpleMoveAction,
                                   SimpleMoveResult,
                                   SimpleMoveFeedback)
from samplereturn_msgs.msg import (VFHMoveGoal,
                                   VFHMoveAction,
                                   VFHMoveResult,
                                   VFHMoveFeedback)

class LevelTwoWeb(object):
    
    def __init__(self):
        
        rospy.on_shutdown(self.shutdown_cb)
        node_params = util.get_node_params()
        self.tf_listener = tf.TransformListener()

        self.world_fixed_frame = rospy.get_param("world_fixed_frame", "map")
        self.odometry_frame = rospy.get_param("odometry_frame", "odom")
        
        #make a Point msg out of beacon_approach_point, and a pose, at that point, facing beacon
        header = std_msg.Header(0, rospy.Time(0), self.world_fixed_frame)
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
                                                       VFHMoveAction)

        self.simple_mover = actionlib.SimpleActionClient("simple_move",
                                                         SimpleMoveAction)

        #get the star shape
        self.spokes = self.get_spokes(node_params.spoke_count,
                                      node_params.max_spoke_length,
                                      node_params.first_spoke_offset,
                                      node_params.spoke_hub_radius,
                                      node_params.next_spoke_sign)
                                                                  
        self.state_machine = smach.StateMachine(
                outcomes=['complete', 'preempted', 'aborted'],
                input_keys = ['action_goal'],
                output_keys = ['action_result'])
    
        #lists
        self.state_machine.userdata.spokes = self.spokes
        self.state_machine.userdata.raster_points = deque()
        
        #store the intial planned goal in map, check for the need to replan
        self.state_machine.userdata.move_point_map = None
        self.replan_threshold = node_params.replan_threshold

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
        
        #search parameters
        self.state_machine.userdata.move_velocity = node_params.move_velocity
        self.state_machine.userdata.spin_velocity = node_params.spin_velocity
        self.state_machine.userdata.spoke_hub_radius = node_params.spoke_hub_radius
        self.state_machine.userdata.next_spoke_sign = node_params.next_spoke_sign
        self.state_machine.userdata.raster_step = node_params.raster_step
        self.state_machine.userdata.raster_offset = node_params.raster_offset
        self.state_machine.userdata.blocked_retry_delay = rospy.Duration(node_params.blocked_retry_delay)
        self.state_machine.userdata.blocked_retried = False
        self.state_machine.userdata.blocked_limit = 2
        self.state_machine.userdata.last_retry_time = rospy.Time.now()

        #web management flags
        self.state_machine.userdata.outbound = True
        self.state_machine.userdata.raster_active = False
        
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
    
        self.state_machine.userdata.vfh_result = None
    
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
                                                 'Enter ing level two mode.  Enable ing search camera.'),
                                   transitions = {'next':'ENABLE_SEARCH_CAMERA'})
            
            smach.StateMachine.add('ENABLE_SEARCH_CAMERA',
                                    smach_ros.ServiceState('enable_search',
                                                            platform_srv.Enable,
                                                            request = platform_srv.EnableRequest(True)),
                                    transitions = {'succeeded':'SELECT_PLANNER',
                                                   'aborted':'LEVEL_TWO_ABORTED'})
            
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
                                   transitions = {'complete':'WEB_MANAGER',
                                                  'sample_detected':'WEB_MANAGER',
                                                  'preempted':'LEVEL_TWO_PREEMPTED',
                                                  'aborted':'LEVEL_TWO_ABORTED'},
                                   remapping = {'stop_on_sample':'false'})
            
            smach.StateMachine.add('WEB_MANAGER',
                                   WebManager(self.tf_listener,
                                              self.announcer),
                                   transitions = {'move':'CREATE_MOVE_GOAL',
                                                  'get_raster':'CREATE_RASTER_POINTS',
                                                  'return_home':'ANNOUNCE_RETURN_HOME',
                                                  'aborted':'LEVEL_TWO_ABORTED'})

            smach.StateMachine.add('CREATE_RASTER_POINTS',
                                   CreateRasterPoints(self.tf_listener),
                                   transitions = {'next':'WEB_MANAGER',
                                                  'aborted':'LEVEL_TWO_ABORTED'})

            smach.StateMachine.add('CREATE_MOVE_GOAL',
                                   CreateMoveGoal(self.tf_listener),
                                   transitions = {'next':'MOVE',
                                                  'aborted':'LEVEL_TWO_ABORTED'})

            smach.StateMachine.add('MOVE',
                                   ExecuteVFHMove(self.vfh_mover),
                                   transitions = {'complete':'WEB_MANAGER',
                                                  'missed_target':'ANNOUNCE_MISSED_TARGET',
                                                  'blocked':'RETRY_CHECK',
                                                  'started_blocked':'RETRY_CHECK',
                                                  'off_course':'ANNOUNCE_OFF_COURSE',
                                                  'sample_detected':'PURSUE_SAMPLE',
                                                  'preempted':'LEVEL_TWO_PREEMPTED',
                                                  'aborted':'LEVEL_TWO_ABORTED'},
                                   remapping = {'stop_on_sample':'true'})
            
            smach.StateMachine.add('ANNOUNCE_BLOCKED',
                                   AnnounceState(self.announcer, 'Path Blocked.'),
                                   transitions = {'next':'WEB_MANAGER'})
            
            smach.StateMachine.add('ANNOUNCE_ROTATE_TO_CLEAR',
                                   AnnounceState(self.announcer, 'Started Blocked.'),
                                   transitions = {'next':'WEB_MANAGER'})

            smach.StateMachine.add('RETRY_CHECK',
                                   RetryCheck(self.announcer),
                                   transitions = {'continue':'WEB_MANAGER',
                                                  'retry':'MOVE',
                                                  'preempted':'LEVEL_TWO_PREEMPTED',
                                                  'aborted':'LEVEL_TWO_ABORTED'})
            
            smach.StateMachine.add('ANNOUNCE_OFF_COURSE',
                                   AnnounceState(self.announcer, 'Off Course.'),
                                   transitions = {'next':'WEB_MANAGER'})
            
            smach.StateMachine.add('ANNOUNCE_MISSED_TARGET',
                                   AnnounceState(self.announcer, 'Missed Target'),
                                   transitions = {'next':'WEB_MANAGER'})

            
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
                                   transitions = {'next':'WEB_MANAGER'})   

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
                                                  'aborted':'LEVEL_TWO_ABORTED'},
                                   remapping = {'stop_on_sample':'stop_on_beacon',
                                                'detected_sample':'beacon_point'})
            
            #return to start along the approach point
            #if the path is ever blocked just give up and return to the level_two search
            smach.StateMachine.add('BEACON_SEARCH_MOVE',
                                   ExecuteVFHMove(self.vfh_mover),
                                   transitions = {'complete':'BEACON_SEARCH',
                                                  'blocked':'BEACON_SEARCH',
                                                  'started_blocked':'BEACON_SEARCH',
                                                  'missed_target':'BEACON_SEARCH',
                                                  'off_course':'BEACON_SEARCH',
                                                  'sample_detected':'BEACON_SEARCH',
                                                  'preempted':'BEACON_SEARCH',
                                                  'aborted':'LEVEL_TWO_ABORTED'},
                                   remapping = {'stop_on_sample':'stop_on_beacon',
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
        
        #start a timer loop to check for localization updates
        rospy.Timer(rospy.Duration(5.0), self.localization_check)
        
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
        #first, update the beacon point
        beacon_point = geometry_msg.PointStamped(beacon_pose.header,
                                                 beacon_pose.pose.pose.position)
        self.state_machine.userdata.beacon_point = beacon_point
    
    def localization_check(self, event):        
        
        saved_point_map = self.state_machine.userdata.move_point_map
        
        #if the VFH server is active, and the beacon correction is large enough, change the goal
        if saved_point_map is not None:

            goal_point_odom = self.state_machine.userdata.move_goal.target_pose.pose.position

            header = std_msg.Header(0, rospy.Time(0), self.world_fixed_frame)
            
            #take the current planned point, and transform it back to map, and
            #compare with the map position last time we planned/corrected
            try:
                self.tf_listener.waitForTransform(self.odometry_frame,
                                                  self.world_fixed_frame,
                                                  rospy.Time(0),
                                                  rospy.Duration(1.0))
                saved_point_odom = self.tf_listener.transformPoint(self.odometry_frame, saved_point_map)
            except tf.Exception:
                rospy.logwarn("LEVEL_TWO beacon_update failed to transform target point")
            
            correction_error = util.point_distance_2d(goal_point_odom,
                                                      saved_point_odom.point)
            
            #rospy.loginfo("CORRECTION ERROR: {:f}".format(correction_error))
            
            if (correction_error > self.replan_threshold):
                self.announcer.say("Beacon correction.")
                #update the VFH move goal
                goal = deepcopy(self.state_machine.userdata.move_goal)
                goal.target_pose.pose.position = saved_point_odom.point
                self.state_machine.userdata.move_goal = goal                

    def get_spokes(self, spoke_count, spoke_length, offset, hub_radius, direction):
        #This creates a list of dictionaries.  The yaw entry is a useful piece of information
        #for some of the state machine states.  The other entry contains the start and end
        #points defined by the line yaw
        
        offset = np.radians(offset)
        yaws = list(np.linspace(0 + offset,
                                direction*2*np.pi + offset,
                                spoke_count,
                                endpoint=False))
        #add the starting spoke again, for raster calcs
        yaws.append(offset)
        spokes = deque()
        for yaw in yaws:
            yaw = util.unwind(yaw)
            header = std_msg.Header(0, rospy.Time(0), self.world_fixed_frame)
            pos = geometry_msg.Point(hub_radius*math.cos(yaw),
                                     hub_radius*math.sin(yaw), 0)
            start_point = geometry_msg.PointStamped(header, pos)
            pos = geometry_msg.Point(spoke_length*math.cos(yaw),
                                     spoke_length*math.sin(yaw), 0)    
            end_point = geometry_msg.PointStamped(header, pos)
            spokes.append({'yaw':yaw,
                           'start_point':start_point,
                           'end_point':end_point})
        
        return spokes
    
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

class WebManager(smach.State):
    def __init__(self, tf_listener, announcer):
        smach.State.__init__(self,
                             input_keys = ['spoke_yaw',
                                           'outbound',
                                           'spokes',
                                           'raster_active',
                                           'raster_points',
                                           'raster_step',
                                           'vfh_result',
                                           'world_fixed_frame'],
                             output_keys = ['spoke_yaw',
                                            'outbound',
                                            'spokes',
                                            'raster_active',
                                            'raster_points',
                                            'retry_active',
                                            'move_point',
                                            'course_tolerance',
                                            'stop_on_sample'],
                             outcomes=['move',
                                       'get_raster',
                                       'return_home',
                                       'preempted', 'aborted'])
        
        self.tf_listener = tf_listener
        self.announcer = announcer
        self.last_raster_move = None

    def execute(self, userdata):
        
        userdata.retry_active = False

        if userdata.outbound:
            #this flag indicates it's time to make an outbound move
            spoke = userdata.spokes.popleft()
            #there is an extra spoke at the end
            if len(userdata.spokes) == 0:
                rospy.loginfo("SPOKE MANAGER finished last spoke")
                return 'return_home'            
            userdata.spoke_yaw = spoke['yaw']
            userdata.move_point = spoke['end_point']
            userdata.outbound = False
            userdata.course_tolerance = 5.0
            rospy.loginfo("WEB_MANAGER starting spoke: %.2f" %(userdata.spoke_yaw))
            self.announcer.say("Start ing on spoke, Yaw %s" % (int(math.degrees(userdata.spoke_yaw))))
            return 'move'
        else:
            #we are inbound
            if userdata.raster_active:
                #still rastering
                next_move = userdata.raster_points.popleft()
                                
                #was the last move a chord, and we got blocked?
                if self.last_raster_move is not None and \
                        userdata.vfh_result in (VFHMoveResult.BLOCKED,
                                                VFHMoveResult.OFF_COURSE) and \
                        self.last_raster_move['radius'] != -1: 
                    robot_yaw = util.get_robot_yaw_from_origin(self.tf_listener,
                                                                userdata.world_fixed_frame)
                    robot_radius = util.get_robot_distance_to_origin(self.tf_listener,
                                                                     userdata.world_fixed_frame)
                    rospy.loginfo("WEB_MANAGER heading inward after blocked chord")
                    
                    #prune the raster until we go to the next chord inside our radius
                    #while robot_radius < next_move['radius']:
                        #are there more points?
                    #    if (len(userdata.raster_points) > 0):
                    #        rospy.loginfo("WEB_MANAGER pruning raster.  Robot radius: {:f}, \
                    #                      next_move[radius]: {:f}, userdata.raster_points[0]: {!s} \
                    #                      ".format(robot_radius,
                    #                               next_move['radius'],
                    #                               userdata.raster_points[0]))
                    #        next_move = userdata.raster_points.popleft()
                    #    else:                            
                    #        break #no more points, just continue
                    
                    x = next_move['radius']*math.cos(robot_yaw)
                    y = next_move['radius']*math.sin(robot_yaw)
                    header = std_msg.Header(0, rospy.Time(0), userdata.world_fixed_frame)
                    pos = geometry_msg.Point(x,y,0)
                    #replace the next inward point with this
                    next_move['point'] = geometry_msg.PointStamped(header, pos)
                    self.announcer.say("Heading inward from short chord.")
                
                #load the target into move_point, and save the move
                self.last_raster_move = next_move
                userdata.move_point = next_move['point']
                
                #use tighter course tolerance for chord moves (no point in getting into next chord)
                if next_move['inward']:
                    userdata.course_tolerance = 5.0
                else:
                    userdata.course_tolerance = 3.0
                
                #is this the last point?
                if len(userdata.raster_points) == 0:
                    userdata.raster_active = False
                    userdata.outbound = True #request the next spoke move
                return 'move'
            else:
                #time to start rastering, create the points, and set flag
                self.announcer.say("Begin ing raster on spoke, Yaw {!s}".format(int(math.degrees(userdata.spoke_yaw))))
                userdata.raster_active = True
                self.last_raster_move = None
                return 'get_raster'
        
        return 'aborted'

class CreateRasterPoints(smach.State):
    def __init__(self, tf_listener):
        smach.State.__init__(self,
                             input_keys = ['spokes',
                                           'spoke_yaw',
                                           'spoke_hub_radius',
                                           'raster_step',
                                           'raster_offset',
                                           'next_spoke_sign',
                                           'world_fixed_frame'],
                             output_keys = ['raster_points'],
                             outcomes=['next',
                                       'preempted', 'aborted'])

        self.tf_listener = tf_listener

    def execute(self, userdata):
            
            raster_points = deque()
            header = std_msg.Header(0, rospy.Time(0), userdata.world_fixed_frame)
            radius = util.get_robot_distance_to_origin(self.tf_listener,
                                                       userdata.world_fixed_frame)
                       
            current_yaw = userdata.spoke_yaw
            next_yaw = userdata.spokes[0]['yaw']
            yaw_step = np.abs(util.unwind(next_yaw - current_yaw))
            raster_offset = userdata.raster_offset
            #get the sign of the rotation to next yaw
            spoke_sign = userdata.next_spoke_sign

            def raster_point(yaw, offset_sign, inward): 
                offset_yaw = util.unwind(yaw + offset_sign*raster_offset/radius*spoke_sign)
                x = radius*math.cos(offset_yaw)
                y = radius*math.sin(offset_yaw)
                point = geometry_msg.PointStamped(header, geometry_msg.Point(x,y,0))
                return {'point':point,'radius':radius,'inward':inward}

            def narrow_check():
                if (yaw_step*radius) < (3.0*userdata.raster_offset):
                    raster_points.append({'point':userdata.spokes[0]['start_point'],
                                          'radius':userdata.spoke_hub_radius,
                                          'inward':True})
                    return True
                else:
                    return False
            
            rospy.loginfo("STARTING RADIUS, YAW, NEXT YAW: {!s}, {!s}, {!s}".format(radius,
                                                                                    current_yaw,
                                                                                    next_yaw))
            
            #generate one ccw-inward-cw-inward set of points per loop
            while True:
                #chord move to next yaw
                raster_points.append(raster_point(next_yaw, -1, False))
                radius -= userdata.raster_step
                if (radius <= userdata.spoke_hub_radius): 
                    #within the hub radius, we're done
                    break
                #inward move on next yaw
                raster_points.append(raster_point(next_yaw, -1, True))
                #if the next chord move < raster offset, go to next spoke
                if narrow_check(): break
                #chord move to current yaw
                raster_points.append(raster_point(current_yaw, 1, False))               
                radius -= userdata.raster_step
                #inward move on current yaw
                raster_points.append(raster_point(current_yaw, 1, True))                              
                #if the next chord move < raster offset, go to next spoke
                if narrow_check(): break
               
            userdata.raster_points = raster_points
            
            return 'next'
    

#take a point and create a VFH goal        
class CreateMoveGoal(smach.State):
    def __init__(self, tf_listener):
        smach.State.__init__(self,
                             input_keys = ['move_point',
                                           'move_velocity',
                                           'spin_velocity',
                                           'course_tolerance',
                                           'vfh_result',
                                           'odometry_frame'],
                             output_keys = ['move_goal',
                                            'line_points',
                                            'next_line_point',
                                            'move_point_map'],
                             outcomes=['next',
                                       'preempted', 'aborted'])
        
        self.tf_listener = tf_listener

    def execute(self, userdata):

        odometry_frame = userdata.odometry_frame
        target_point = userdata.move_point
        rotate_to_clear = False
   
        #if we are back here after starting a move in the blocked condition, try rotate to clear
        if userdata.vfh_result == VFHMoveResult.STARTED_BLOCKED:
                rotate_to_clear = True    
        
        try:
            self.tf_listener.waitForTransform(odometry_frame,
                                              target_point.header.frame_id,
                                              target_point.header.stamp,
                                              rospy.Duration(2.0))
            point_in_odom = self.tf_listener.transformPoint(odometry_frame,
                                                            target_point)
        except tf.Exception:
            rospy.logwarn("LEVEL_TWO failed to transform goal point %s->%s",
                          target_point.header.frame_id, odometry_frame)
            return 'aborted'

        header = std_msg.Header(0, rospy.Time(0), odometry_frame)
        pose = geometry_msg.Pose(position = point_in_odom.point,
                                 orientation = geometry_msg.Quaternion())
        target_pose = geometry_msg.PoseStamped(header, pose)
        goal = VFHMoveGoal(target_pose = target_pose,
                           move_velocity = userdata.move_velocity,
                           spin_velocity = userdata.spin_velocity,
                           course_tolerance = userdata.course_tolerance,
                           orient_at_target = False,
                           rotate_to_clear = rotate_to_clear)
        userdata.move_goal = goal

        #store the point in map, to compare in the beacon update callback
        userdata.move_point_map = target_point
        
        return 'next'

#For retrying a move after line_blocked.  Probably a terrible hack that should be removed later.
#Wait for the delay, then try to move again.  Allow retry after delay time has elapsed since last retry also
class RetryCheck(smach.State):
            
    def __init__(self, announcer):           
        smach.State.__init__(self,
                             input_keys = ['retry_active',
                                           'blocked_retry_delay',
                                           'vfh_result',
                                           'move_goal'],
                             output_keys = ['retry_active',
                                            'vfh_result',
                                            'move_goal'],
                             outcomes=['retry',
                                       'continue',
                                       'preempted', 'aborted'])
        
        self.announcer = announcer  

    def execute(self, userdata):
        
        if userdata.retry_active and (userdata.vfh_result == VFHMoveResult.STARTED_BLOCKED):
            #if we are retrying a move, starting blocked means the costmap is correct
            #and we are blocked, so change the STARTED_BLOCKED result to regular old BLOCKED
            userdata.vfh_result = VFHMoveResult.BLOCKED
            return 'continue'
                   
        self.announcer.say('Recheck ing cost map.')
        rospy.sleep(userdata.blocked_retry_delay)
        userdata.retry_active = True
        return 'retry'
            
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
                                         'world_fixed_frame',
                                         'odometry_frame'],
                             output_keys=['move_goal',
                                          'move_point_map',
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
        
        #ignore samples after this (applies to mount moves)
        #clear previous move_point_map
        userdata.stop_on_sample = False
        userdata.move_point_map = None
        map_header = std_msg.Header(0, rospy.Time(0), userdata.world_fixed_frame)
       
        #get our position in map
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
                userdata.move_point_map = geometry_msg.PointStamped(map_header,
                                                                    userdata.beacon_approach_pose.pose.position)
                goal = VFHMoveGoal(target_pose = userdata.beacon_approach_pose,
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
                #save point
                userdata.move_point_map = geometry_msg.PointStamped(map_header,
                                                    search_pose.pose.position)
                goal = VFHMoveGoal(target_pose = userdata.search_pose,
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
                goal = VFHMoveGoal(target_pose = front_pose,
                                   move_velocity = userdata.move_velocity,
                                   spin_velocity = userdata.spin_velocity)
                userdata.move_goal = goal                          
                self.announcer.say("Back of beacon in view. Move ing to front")
                return 'move'   
            elif distance_to_approach_point > 2.0:
                #on correct side of beacon, but far from approach point
                goal = VFHMoveGoal(target_pose = userdata.beacon_approach_pose,
                                   move_velocity = userdata.move_velocity,
                                   spin_velocity = userdata.spin_velocity,
                                   orient_at_target = True)
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
    

    
    
    
