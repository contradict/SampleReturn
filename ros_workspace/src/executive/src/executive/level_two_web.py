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
import rosparam
import actionlib
import tf

import std_msgs.msg as std_msg
import actionlib_msgs.msg as action_msg
import nav_msgs.msg as nav_msg
import visual_servo_msgs.msg as visual_servo_msg
import samplereturn_msgs.msg as samplereturn_msg
import samplereturn_msgs.srv as samplereturn_srv
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
from executive.beacon_search_states import BeaconSearch
from executive.beacon_search_states import MountManager

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
    
        #move management
        self.state_machine.userdata.vfh_result = None
        self.state_machine.userdata.active_manager = None
        #list of manager outcomes, and their corresponding labels
        manager_dict = {'WEB_MANAGER':'web_manager',
                        'RETURN_MANAGER':'return_manager',
                        'RECOVERY_MANAGER':'recovery_manager'}
        self.state_machine.userdata.manager_dict = manager_dict
        #get inverted version for the transition dict of the mux
        manager_transitions = dict([[v,k] for k,v in manager_dict.items()])
    
        #motion mode stuff
        planner_mode = node_params.planner_mode
        MODE_PLANNER = getattr(platform_srv.SelectMotionModeRequest, planner_mode)
        MODE_SERVO = platform_srv.SelectMotionModeRequest.MODE_SERVO
        MODE_PAUSE = platform_srv.SelectMotionModeRequest.MODE_PAUSE
    
        with self.state_machine:
            
            smach.StateMachine.add('START_LEVEL_TWO',
                                   StartLeveLTwo(input_keys=['dismount_move'],
                                                 output_keys=['action_result',
                                                              'stop_on_sample',
                                                              'pursue_samples',
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
                                   WebManager('WEB_MANAGER',
                                              self.tf_listener,
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
                                   transitions = {'complete':'MOVE_MUX',
                                                  'missed_target':'ANNOUNCE_MISSED_TARGET',
                                                  'blocked':'RETRY_CHECK',
                                                  'started_blocked':'RETRY_CHECK',
                                                  'off_course':'ANNOUNCE_OFF_COURSE',
                                                  'sample_detected':'PURSUE_SAMPLE',
                                                  'preempted':'LEVEL_TWO_PREEMPTED',
                                                  'aborted':'LEVEL_TWO_ABORTED'})

            smach.StateMachine.add('RETRY_CHECK',
                                   RetryCheck(self.announcer),
                                   transitions = {'continue':'MOVE_MUX',
                                                  'retry':'CREATE_MOVE_GOAL',
                                                  'preempted':'LEVEL_TWO_PREEMPTED',
                                                  'aborted':'LEVEL_TWO_ABORTED'})
            
            smach.StateMachine.add('ANNOUNCE_BLOCKED',
                                   AnnounceState(self.announcer, 'Path Blocked.'),
                                   transitions = {'next':'MOVE_MUX'})
            
            smach.StateMachine.add('ANNOUNCE_ROTATE_TO_CLEAR',
                                   AnnounceState(self.announcer, 'Started Blocked.'),
                                   transitions = {'next':'MOVE_MUX'})
            
            smach.StateMachine.add('ANNOUNCE_OFF_COURSE',
                                   AnnounceState(self.announcer, 'Off Course.'),
                                   transitions = {'next':'MOVE_MUX'})
            
            smach.StateMachine.add('ANNOUNCE_MISSED_TARGET',
                                   AnnounceState(self.announcer, 'Missed Target'),
                                   transitions = {'next':'MOVE_MUX'})

            smach.StateMachine.add('MOVE_MUX',
                                   MoveMUX(manager_transitions.keys()),
                                   transitions = manager_transitions)
            
            @smach.cb_interface(input_keys=['detected_sample'],
                                output_keys=['move_point_map'])
            def pursuit_goal_cb(userdata, request):
                goal = samplereturn_msg.GeneralExecutiveGoal()
                goal.input_point = userdata.detected_sample
                goal.input_string = "level_two_pursuit_request"
                #disable localization checks while in pursuit
                userdata.move_point_map = None
                return goal
            
            @smach.cb_interface(output_keys=['detected_sample'])
            def pursuit_result_cb(userdata, status, result):
                #clear samples after a pursue action
                rospy.sleep(2.0) #wait 2 seconds for detector/filter to clear for sure
                userdata.detected_sample = None
                                            
            smach.StateMachine.add('PURSUE_SAMPLE',
                                  smach_ros.SimpleActionState('pursue_sample',
                                  samplereturn_msg.GeneralExecutiveAction,
                                  goal_cb = pursuit_goal_cb,
                                  result_cb = pursuit_result_cb),
                                  transitions = {'succeeded':'ANNOUNCE_RETURN_TO_SEARCH',
                                                 'aborted':'ANNOUNCE_RETURN_TO_SEARCH'})

            smach.StateMachine.add('ANNOUNCE_RETURN_TO_SEARCH',
                                   AnnounceState(self.announcer,
                                                 'Return ing to search.'),
                                   transitions = {'next':'CREATE_MOVE_GOAL'})   

            smach.StateMachine.add('ANNOUNCE_RETURN_HOME',
                                   AnnounceState(self.announcer,
                                                 'Return ing to home platform.'),
                                   transitions = {'next':'RETURN_MANAGER'})
 
            smach.StateMachine.add('RETURN_MANAGER',
                                   BeaconSearch('RETURN_MANAGER',
                                                self.tf_listener,
                                                self.announcer),
                                   transitions = {'mount':'MOUNT_MANAGER',
                                                  'move':'CREATE_MOVE_GOAL',
                                                  'spin':'BEACON_SEARCH_SPIN',
                                                  'aborted':'LEVEL_TWO_ABORTED'})
 
            smach.StateMachine.add('BEACON_SEARCH_SPIN',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'RETURN_MANAGER',
                                                  'sample_detected':'RETURN_MANAGER',
                                                  'preempted':'LEVEL_TWO_PREEMPTED',
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
            
            smach.StateMachine.add('RECOVERY_MANAGER',
                                   RecoveryManager('RECOVERY_MANAGER',
                                                   self.announcer,
                                                   self.tf_listener),
                                   transitions = {'move':'CREATE_MOVE_GOAL',
                                                  'beacon_search':'RETURN_MANAGER',
                                                  'web_manager':'WEB_MANAGER'})
    
            smach.StateMachine.add('LEVEL_TWO_PREEMPTED',
                                  LevelTwoPreempted(),
                                   transitions = {'recovery':'RECOVERY_MANAGER',
                                                  'exit':'preempted'})
            
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
        
        rospy.Service("start_recovery", samplereturn_srv.RecoveryParameters, self.start_recovery)
        
        #start a timer loop to check for localization updates
        rospy.Timer(rospy.Duration(5.0), self.localization_check)
        
        #start action servers and services
        sls.start()
        level_two_server.run_server()
        rospy.spin()
        sls.stop()
    
    def start_recovery(self, req):
        try:
            params = rosparam.load_file(req.file)
            rospy.loginfo("START_RECOVERY received params: {!s}".format(params))
            #get first namespace, and the first tuple entry (the param dict)
            params = params[0][0]
        except Exception, exc:
            rospy.loginfo("START_RECOVERY failed to parse params: {!s}".format(exc))
            return False
            
        self.state_machine.userdata.recovery_requested = True
        self.state_machine.userdata.recovery_parameters = params
        self.state_machine.request_preempt()
        return True
    
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
            
            rospy.loginfo("CORRECTION ERROR: {:f}".format(correction_error))
            
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
        
        userdata.pursue_samples = True
        userdata.stop_on_sample = True

        return 'next'

class WebManager(smach.State):
    def __init__(self, label, tf_listener, announcer):
        smach.State.__init__(self,
                             input_keys = ['spoke_yaw',
                                           'outbound',
                                           'spokes',
                                           'raster_active',
                                           'raster_points',
                                           'raster_step',
                                           'vfh_result',
                                           'manager_dict',
                                           'return_time',
                                           'world_fixed_frame'],
                             output_keys = ['spoke_yaw',
                                            'outbound',
                                            'spokes',
                                            'raster_active',
                                            'raster_points',
                                            'retry_active',
                                            'web_move',
                                            'move_target',
                                            'course_tolerance',
                                            'stop_on_sample',
                                            'pursue_samples',
                                            'active_manager'],
                             outcomes=['move',
                                       'get_raster',
                                       'return_home',
                                       'preempted', 'aborted'])
        
        self.tf_listener = tf_listener
        self.announcer = announcer
        self.label = label
        self.last_web_move = None
        self.blocked_set = [VFHMoveResult.BLOCKED, VFHMoveResult.OFF_COURSE]

    def execute(self, userdata):
        #set the move manager key for the move mux
        userdata.active_manager = userdata.manager_dict[self.label]        
        
        if rospy.Time.now() > userdata.return_time:
            userdata.stop_on_sample = False
            userdata.pursue_samples = False
            return 'return_home'

        if userdata.outbound:
            #this flag indicates it's time to make an outbound move
            spoke = userdata.spokes.popleft()
            #there is an extra spoke at the end
            if len(userdata.spokes) == 0:
                rospy.loginfo("SPOKE MANAGER finished last spoke")
                return 'return_home'
            userdata.spoke_yaw = spoke['yaw']
            next_move = {'point':spoke['end_point'], 'radius':0, 'radial':True}
            userdata.outbound = False
            rospy.loginfo("WEB_MANAGER starting spoke: %.2f" %(userdata.spoke_yaw))
            remaining_minutes = int((userdata.return_time - rospy.Time.now()).to_sec()/60)
            self.announcer.say("Start ing on spoke, yaw {!s}.  {:d} minutes left".format(int(math.degrees(userdata.spoke_yaw)),
                                                                                         remaining_minutes))
            return self.load_move(next_move, userdata)
        else:
            #we are inbound
            if userdata.raster_active:
                #still rastering
                next_move = userdata.raster_points.popleft()
                #was the last move a chord, and we got blocked?
                if (userdata.vfh_result in self.blocked_set) and \
                        not self.last_web_move['radial']: 
                    robot_yaw = util.get_robot_yaw_from_origin(self.tf_listener,
                                                                userdata.world_fixed_frame)
                    robot_radius = util.get_robot_distance_to_origin(self.tf_listener,
                                                                     userdata.world_fixed_frame)
                    rospy.loginfo("WEB_MANAGER heading to next chord after blocked chord")
                    x = next_move['radius']*math.cos(robot_yaw)
                    y = next_move['radius']*math.sin(robot_yaw)
                    header = std_msg.Header(0, rospy.Time(0), userdata.world_fixed_frame)
                    pos = geometry_msg.Point(x,y,0)
                    #replace the next inward point with this
                    next_move['point'] = geometry_msg.PointStamped(header, pos)
                    self.announcer.say("Head ing to next radius.")
                                
                #is this the last point?
                if len(userdata.raster_points) == 0:
                    self.announcer.say("Return ing from raster on spoke, yaw {!s}".format(int(math.degrees(userdata.spoke_yaw))))
                    userdata.raster_active = False
                    userdata.outbound = True #request the next spoke move
                
                return self.load_move(next_move, userdata)
            else:
                #time to start rastering, create the points, and set flag
                self.announcer.say("Begin ing raster on spoke, yaw {!s}".format(int(math.degrees(userdata.spoke_yaw))))
                userdata.raster_active = True
                return 'get_raster'
        
        return 'aborted'
    
    def load_move(self, next_move, userdata):
        #use tighter course tolerance for chord moves (no point in getting into next chord)
        if next_move['radial']:
            userdata.course_tolerance = 5.0
        else:
            userdata.course_tolerance = 3.0
        #load the target into move_point, and save the move
        #move_point is consumed by the general move_goal transformer,
        #and web_move is used to do retry_checks, etc.
        self.last_web_move = next_move
        userdata.web_move = next_move
        userdata.move_target = next_move['point']
    
        return 'move'

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
        
        self.debug_marker_pub = rospy.Publisher('web_markers',
                                                vis_msg.MarkerArray,
                                                queue_size=3)
        self.marker_id = 0

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
                return {'point':point,'radius':radius,'radial':inward}

            def too_narrow():
                if (yaw_step*radius) < (2*userdata.raster_offset + 2.0):
                    raster_points.append({'point':userdata.spokes[0]['start_point'],
                                          'radius':userdata.spoke_hub_radius,
                                          'radial':True})
                    return True
                else:
                    return False
            
            rospy.loginfo("STARTING RADIUS, YAW, NEXT YAW: {!s}, {!s}, {!s}".format(radius,
                                                                                    current_yaw,
                                                                                    next_yaw))
            
            #generate one ccw-inward-cw-inward set of points per loop
            #if the next chord move < raster offset, go to next spoke
            while not too_narrow():
                #chord move to next yaw
                raster_points.append(raster_point(next_yaw, -1, False))
                radius -= userdata.raster_step
                #inward move on next yaw
                raster_points.append(raster_point(next_yaw, -1, True))
                #if the next chord move < raster offset, go to next spoke
                if too_narrow(): break
                #chord move back to current yaw
                raster_points.append(raster_point(current_yaw, 1, False))               
                radius -= userdata.raster_step
                #inward move on current yaw
                raster_points.append(raster_point(current_yaw, 1, True))                              
                 
            #debug points
            debug_array = []
            for raster_point in raster_points:
                debug_marker = vis_msg.Marker()
                debug_marker.header = header
                debug_marker.type = vis_msg.Marker.CYLINDER
                debug_marker.color = std_msg.ColorRGBA(1, 1, 0, 1)
                debug_marker.scale = geometry_msg.Vector3(.5, .5, .5)
                debug_marker.lifetime = rospy.Duration(0)                
                debug_marker.pose.position = raster_point['point'].point
                debug_marker.id = self.marker_id
                debug_array.append(debug_marker)
                self.marker_id += 1                  
                
            self.debug_marker_pub.publish(vis_msg.MarkerArray(debug_array))          
          
            userdata.raster_points = raster_points
            return 'next'

#take a point and create a VFH goal        
class CreateMoveGoal(smach.State):
    def __init__(self, tf_listener):
        smach.State.__init__(self,
                             input_keys = ['move_target',
                                           'move_velocity',
                                           'spin_velocity',
                                           'course_tolerance',
                                           'vfh_result',
                                           'odometry_frame',
                                           'world_fixed_frame'],
                             output_keys = ['move_goal',
                                            'move_point_map'],
                             outcomes=['next',
                                       'preempted', 'aborted'])
        
        self.tf_listener = tf_listener

    def execute(self, userdata):

        odometry_frame = userdata.odometry_frame
        target = userdata.move_target
        rotate_to_clear = False
        orient_at_target = False
   
        #if we are back here after starting a move in the blocked condition, try rotate to clear
        if userdata.vfh_result == VFHMoveResult.STARTED_BLOCKED:
            rospy.loginfo("LEVEL_TWO setting rotate_to_clear = True")
            rotate_to_clear = True    
        
        try:
            #do all movement transforms with latest information
            target.header.stamp = rospy.Time(0) 
            self.tf_listener.waitForTransform(odometry_frame,
                                              target.header.frame_id,
                                              target.header.stamp,
                                              rospy.Duration(2.0))
            if isinstance(target, geometry_msg.PoseStamped):
                target_pose = self.tf_listener.transformPose(odometry_frame, target)                 
                orient_at_target = True
            elif isinstance(target, geometry_msg.PointStamped):
                pt_odom = self.tf_listener.transformPoint(odometry_frame, target)
                pose = geometry_msg.Pose(position = pt_odom.point,
                                 orientation = geometry_msg.Quaternion())
                target_pose = geometry_msg.PoseStamped(pt_odom.header, pose)
            else:
                rospy.logwarn("LEVEL_TWO invalid move target in CreateMoveGoal")
                return 'aborted'
            
            #input point could be in any frame (definitely baselink sometimes)
            map_pose = self.tf_listener.transformPose(userdata.world_fixed_frame,
                                                      target_pose)
                
        except tf.Exception, exc:
            rospy.logwarn("LEVEL_TWO failed to transform move: {!s}".format(exc))
            return 'aborted'
        
        goal = VFHMoveGoal(target_pose = target_pose,
                           move_velocity = userdata.move_velocity,
                           spin_velocity = userdata.spin_velocity,
                           course_tolerance = userdata.course_tolerance,
                           orient_at_target = orient_at_target,
                           rotate_to_clear = rotate_to_clear)
        userdata.move_goal = goal

        #store the point in map, to compare in the beacon update callback
        userdata.move_point_map = geometry_msg.PointStamped(map_pose.header,
                                                            map_pose.pose.position)
        
        
        return 'next'

#For retrying a move after line_blocked.  Probably a terrible hack that should be removed later.
#Wait for the delay, then try to move again.  Allow retry after delay time has elapsed since last retry also
class RetryCheck(smach.State):
            
    def __init__(self, announcer):           
        smach.State.__init__(self,
                             input_keys = ['retry_active',
                                           'blocked_retry_delay',
                                           'web_move',
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
        
        #we only get here if BLOCKED or STARTED_BLOCKED
        if userdata.vfh_result == VFHMoveResult.STARTED_BLOCKED:
            #If we are retrying a move, starting blocked means the costmap is correct
            #and we are blocked, so change the STARTED_BLOCKED result to regular old BLOCKED
            #Also, if we are making a chord move just return with blocked
            if userdata.retry_active or not userdata.web_move['radial']:
                userdata.vfh_result = VFHMoveResult.BLOCKED
                return 'continue'
            else:
                self.announcer.say('Rotate to clear')
                userdata.retry_active = True
                return 'retry'
        
        #If we got here as a result of BLOCKED, movement occured
        #since last time. So, go ahead and retry
        self.announcer.say('Recheck ing cost map.')
        rospy.sleep(userdata.blocked_retry_delay)
        userdata.retry_active = True
        return 'retry'
    

class RecoveryManager(smach.State):
    def __init__(self, label, announcer, tf_listener):
        smach.State.__init__(self,
                             input_keys = ['recovery_parameters',
                                           'recovery_requested',
                                           'manager_dict',
                                           'raster_points'],
                             output_keys = ['recovery_parameters',
                                            'recovery_requested',
                                            'active_manager',
                                            'raster_points',
                                            'move_target',
                                            'pursue_samples'],
                             outcomes=['move',
                                       'beacon_search',
                                       'web_manager',
                                       'aborted'])
        
        self.label = label       
        self.announcer = announcer
        self.tf_listener = tf_listener
        
    def execute(self, userdata):
        #set the move manager key for the move mux
        userdata.active_manager = userdata.manager_dict[self.label]
        
        rospy.loginfo("RECOVERY_MANAGER starting with params: {!s}".format(userdata.recovery_parameters))
        
        if userdata.recovery_requested == True:
            #first time through
            self.announcer.say("Enter ing recovery state")
            userdata.recovery_requested = False
            userdata.pursue_samples = userdata.recovery_parameters['pursue_samples']
            if userdata.recovery_parameters['spokes_to_remove'] > 0:
                if len(userdata.raster_points) > 0:
                    userdata.raster_points = [userdata.raster_points[-1]]
        
        if len(userdata.recovery_parameters['moves']) > 0:
            move = userdata.recovery_parameters['moves'].pop(0)
            base_header = std_msg.Header(0, rospy.Time(0), 'base_link')            
            base_pose_stamped = geometry_msg.PoseStamped(header = base_header)
            userdata.move_target = util.pose_translate_by_yaw(base_pose_stamped,
                                                              move['distance'],
                                                              np.radians(move['rotate']))
            return 'move'
        else:
            return userdata.recovery_parameters['terminal_outcome']
        

class MoveMUX(smach.State):
    def __init__(self, manager_list):
        smach.State.__init__(self,
                             input_keys = ['active_manager'],
                             output_keys = ['retry_active'],
                             outcomes = manager_list)
        
    def execute(self, userdata):
        userdata.retry_active = False
        return userdata.active_manager
    
class LevelTwoPreempted(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             input_keys = ['recovery_requested'],
                             outcomes=['recovery','exit'])
               
        
    def execute(self, userdata):
        
        if userdata.recovery_requested:
            
            return 'recovery'
        
        return 'exit'

class LevelTwoAborted(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next'])
        
    def execute(self, userdata):
        
        return 'next'
    

    
    
    
