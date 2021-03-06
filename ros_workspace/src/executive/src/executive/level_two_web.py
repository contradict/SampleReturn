#!/usr/bin/env python
import math
import threading
import random
import yaml
import time
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
import samplereturn_msgs.msg as samplereturn_msg
import samplereturn_msgs.srv as samplereturn_srv
import move_base_msgs.msg as move_base_msg
import geometry_msgs.msg as geometry_msg
import platform_motion_msgs.msg as platform_msg
import platform_motion_msgs.srv as platform_srv
import visualization_msgs.msg as vis_msg
import kvh_fog.srv as kvh_fog_srv

import motion_planning.simple_motion as simple_motion

from executive.executive_states import SelectMotionMode
from executive.executive_states import AnnounceState
from executive.executive_states import ExecuteSimpleMove
from executive.executive_states import ExecuteVFHMove
from executive.executive_states import WaitForFlagState
from executive.beacon_return_states import BeaconReturn
from executive.beacon_return_states import CalculateMountMove

from kvh_fog.srv import MeasureBias, MeasureBiasRequest, MeasureBiasResponse


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
        self.platform_frame =  rospy.get_param("platform_frame", "platform")
        self.local_frame = 'base_link'

        #need platform point
        platform_header = std_msg.Header(0, rospy.Time(0), self.platform_frame)
        platform_point = geometry_msg.Point(node_params.platform_point['x'],
                                            node_params.platform_point['y'], 0)

        self.platform_point = geometry_msg.PointStamped(platform_header, platform_point)

        #make a Point msg out of beacon_approach_point, and a pose, at that point, facing beacon
        point = geometry_msg.Point(node_params.beacon_approach_point['x'],
                                   node_params.beacon_approach_point['y'], 0)
        beacon_facing_orientation = util.pointing_quaternion_2d(point, platform_point)
        pose = geometry_msg.Pose(position = point, orientation = beacon_facing_orientation)
        self.beacon_approach_pose = geometry_msg.PoseStamped(header = platform_header, pose = pose)
        self.last_beacon_point = None
        self.last_correction_error = 0

        #interfaces
        self.announcer = util.AnnouncerInterface("audio_search")
        self.CAN_interface = util.CANInterface()
        self.beacon_enable = rospy.Publisher('enable_beacon',
                                              std_msg.Bool,
                                              queue_size=10)       
        
        self.search_enable = rospy.Publisher('enable_search',
                                              std_msg.Bool,
                                              queue_size=10)

        self.camera_enablers = [self.beacon_enable, self.search_enable]
        
        #movement action servers
        self.vfh_mover = actionlib.SimpleActionClient("vfh_move",
                                                       VFHMoveAction)

        self.simple_mover = actionlib.SimpleActionClient("simple_move",
                                                         SimpleMoveAction)

        self.state_machine = smach.StateMachine(
                outcomes=['complete', 'preempted', 'aborted'],
                input_keys = ['action_goal'],
                output_keys = ['action_result'])

        #lists
        self.web_slices = node_params.web_slices
        self.state_machine.userdata.web_slices = deque(self.web_slices)
        self.state_machine.userdata.web_slice_indices = deque(range(len(self.web_slices)))
        
        self.state_machine.userdata.raster_points = deque()

        #these are important values! master frame id and return timing
        self.state_machine.userdata.world_fixed_frame = self.world_fixed_frame
        self.state_machine.userdata.odometry_frame = self.odometry_frame
        self.state_machine.userdata.local_frame = self.local_frame
        self.state_machine.userdata.platform_frame = self.platform_frame
        self.state_machine.userdata.start_time = rospy.Time.now()
        self.state_machine.userdata.return_time_offset = rospy.Duration(node_params.return_time_minutes*60)
        self.state_machine.userdata.pause_time_offset = rospy.Duration(0)
        self.state_machine.userdata.pause_start_time = None
        self.state_machine.userdata.pre_cached_id = samplereturn_msg.NamedPoint.PRE_CACHED

        #initial behavior
        self.state_machine.userdata.initial_behavior = node_params.initial_behavior
        
        #beacon and localization
        self.state_machine.userdata.beacon_approach_pose = self.beacon_approach_pose
        self.state_machine.userdata.beacon_observation_delay = rospy.Duration(node_params.beacon_observation_delay)
        self.state_machine.userdata.beacon_mount_tolerance = node_params.beacon_mount_tolerance
        self.state_machine.userdata.platform_point = self.platform_point
        self.beacon_calibration_delay = node_params.beacon_calibration_delay
        #store the intial planned goal in map, check for the need to replan
        self.state_machine.userdata.move_point_map = None
        self.replan_threshold = node_params.replan_threshold
        self.recalibrate_threshold = node_params.recalibrate_threshold

        #search parameters
        self.state_machine.userdata.default_velocity = node_params.move_velocity
        self.state_machine.userdata.move_velocity = node_params.move_velocity
        self.state_machine.userdata.raster_velocity = node_params.raster_velocity
        self.state_machine.userdata.spin_velocity = node_params.spin_velocity
        self.state_machine.userdata.course_tolerance = None
        self.state_machine.userdata.chord_course_tolerance = node_params.chord_course_tolerance
        self.state_machine.userdata.spoke_hub_radius = node_params.spoke_hub_radius
        self.state_machine.userdata.raster_step = node_params.raster_step
        self.state_machine.userdata.raster_tolerance = node_params.raster_tolerance
        self.state_machine.userdata.blocked_retry_delay = rospy.Duration(node_params.blocked_retry_delay)

        #web management flags
        self.state_machine.userdata.outbound = True
        self.state_machine.userdata.raster_active = False

        #stop function flags
        self.state_machine.userdata.report_sample = False
        self.state_machine.userdata.report_beacon = False
        self.state_machine.userdata.stop_on_detection = True

        #subscriber controlled userdata
        self.state_machine.userdata.paused = False
        self.state_machine.userdata.detected_sample = None
        self.state_machine.userdata.beacon_point = None
        self.state_machine.userdata.recovery_requested = False
        self.state_machine.userdata.map_calibration_requested = False

        #use these as booleans in remaps
        self.state_machine.userdata.true = True
        self.state_machine.userdata.false = False

        #move management
        self.state_machine.userdata.vfh_result = None
        self.state_machine.userdata.active_manager = None
        self.state_machine.userdata.retry_active = False
        #list of manager outcomes, and their corresponding labels
        manager_dict = {'WEB_MANAGER':'web_manager',
                        'RETURN_MANAGER':'return_manager',
                        'RECOVERY_MANAGER':'recovery_manager',
                        'LEVEL_TWO_PREEMPTED':'preempted'}
        self.state_machine.userdata.manager_dict = manager_dict
        #get inverted version for the transition dict of the mux
        manager_transitions = dict([[v,k] for k,v in manager_dict.items()])

        #motion mode stuff
        planner_mode = node_params.planner_mode
        MODE_PLANNER = getattr(platform_srv.SelectMotionModeRequest, planner_mode)
        MODE_SERVO = platform_srv.SelectMotionModeRequest.MODE_SERVO
        MODE_ENABLE = platform_srv.SelectMotionModeRequest.MODE_ENABLE

        with self.state_machine:

            smach.StateMachine.add('START_LEVEL_TWO',
                                   StartLeveLTwo(),
                                   transitions = {'next':'SELECT_PLANNER'})

            smach.StateMachine.add('ANNOUNCE_LEVEL_TWO',
                                   AnnounceState(self.announcer,
                                                 'Enter ing level two mode.'),
                                   transitions = {'next':'SELECT_PLANNER'})

            smach.StateMachine.add('SELECT_PLANNER',
                                    SelectMotionMode(self.CAN_interface,
                                                     MODE_PLANNER),
                                    transitions = {'next':'RECOVERY_MANAGER',
                                                   'paused':'WAIT_FOR_UNPAUSE',
                                                  'failed':'LEVEL_TWO_ABORTED'})

            smach.StateMachine.add('WAIT_FOR_UNPAUSE',
                                   WaitForFlagState('paused',
                                                    flag_trigger_value = False,
                                                    timeout = 20,
                                                    announcer = self.announcer,
                                                    start_message ='System is paused. Un pause to begin level two'),
                                   transitions = {'next':'SELECT_PLANNER',
                                                  'timeout':'WAIT_FOR_UNPAUSE',
                                                  'preempted':'LEVEL_TWO_PREEMPTED'})

            smach.StateMachine.add('WEB_MANAGER',
                                   WebManager('WEB_MANAGER',
                                              self.tf_listener,
                                              self.beacon_enable,
                                              self.search_enable,
                                              self.announcer),
                                   transitions = {'move':'CREATE_MOVE_GOAL',
                                                  'get_raster':'CREATE_RASTER_POINTS',
                                                  'sample_detected':'PURSUE_SAMPLE',
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
                                                  'object_detected':'MOVE_MUX',
                                                  'preempted':'LEVEL_TWO_PREEMPTED',
                                                  'aborted':'LEVEL_TWO_ABORTED'})

            smach.StateMachine.add('RETRY_CHECK',
                                   RetryCheck(self.announcer),
                                   transitions = {'continue':'MOVE_MUX',
                                                  'retry':'CREATE_MOVE_GOAL',
                                                  'preempted':'LEVEL_TWO_PREEMPTED',
                                                  'aborted':'LEVEL_TWO_ABORTED'})

            smach.StateMachine.add('ANNOUNCE_OFF_COURSE',
                                   AnnounceState(self.announcer, 'Off Course.'),
                                   transitions = {'next':'MOVE_MUX'})

            smach.StateMachine.add('ANNOUNCE_MISSED_TARGET',
                                   AnnounceState(self.announcer, 'Missed Target'),
                                   transitions = {'next':'MOVE_MUX'})

            smach.StateMachine.add('MOVE_MUX',
                                   MoveMUX(manager_transitions.keys()),
                                   transitions = manager_transitions)

            @smach.cb_interface(input_keys=['detection_message'])
            def pursuit_goal_cb(userdata, request):
                goal = samplereturn_msg.GeneralExecutiveGoal()
                goal.input_point = userdata.detection_message
                goal.input_string = "level_two_pursuit_request"
                #disable localization checks while in pursuit
                return goal

            @smach.cb_interface(output_keys=['detection_message'])
            def pursuit_result_cb(userdata, status, result):
                #clear samples after a pursue action
                rospy.sleep(2.0) #wait 2 seconds for detector/filter to clear for sure
                userdata.detection_message = None

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
                                   BeaconReturn('RETURN_MANAGER',
                                                self.tf_listener, self.beacon_enable, self.announcer),
                                   transitions = {'mount':'CALCULATE_MOUNT_MOVE',
                                                  'move':'CREATE_MOVE_GOAL',
                                                  'spin':'BEACON_SEARCH_SPIN',
                                                  'preempted':'LEVEL_TWO_PREEMPTED',
                                                  'aborted':'LEVEL_TWO_ABORTED'},
                                   remapping = {'beacon_point':'detection_message'})

            smach.StateMachine.add('BEACON_SEARCH_SPIN',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'RETURN_MANAGER',
                                                  'object_detected':'RETURN_MANAGER',
                                                  'preempted':'LEVEL_TWO_PREEMPTED',
                                                  'aborted':'LEVEL_TWO_ABORTED'})

            smach.StateMachine.add('CALCULATE_MOUNT_MOVE',
                                   CalculateMountMove(self.tf_listener, self.announcer),
                                   transitions = {'move':'MOUNT_MOVE',
                                                  'aborted':'LEVEL_TWO_ABORTED'})

            smach.StateMachine.add('MOUNT_MOVE',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'WAIT_FOR_PREEMPT',
                                                  'object_detected':'RETURN_MANAGER',
                                                  'preempted':'LEVEL_TWO_PREEMPTED',
                                                  'aborted':'LEVEL_TWO_ABORTED'},
                                   remapping = {'stop_on_detection':'false'})

            smach.StateMachine.add('WAIT_FOR_PREEMPT',
                                   WaitForFlagState('false',
                                                    flag_trigger_value = 'true',
                                                    timeout = 20,
                                                    announcer = self.announcer,
                                                    start_message ='Level two complete.'),
                                   transitions = {'next':'WAIT_FOR_PREEMPT',
                                                  'timeout':'WAIT_FOR_PREEMPT',
                                                  'preempted':'LEVEL_TWO_PREEMPTED'})

            smach.StateMachine.add('RECOVERY_MANAGER',
                                   RecoveryManager('RECOVERY_MANAGER',
                                                   self.announcer,
                                                   self.beacon_enable,
                                                   self.search_enable,
                                                   self.tf_listener),
                                   transitions = {'move':'CREATE_MOVE_GOAL',
                                                  'simple_move':'RECOVERY_SIMPLE_MOVE',
                                                  'sample_detected':'PURSUE_SAMPLE',
                                                  'return_manager':'RETURN_MANAGER',
                                                  'web_manager':'WEB_MANAGER',
                                                  'wait_for_preempt':'WAIT_FOR_PREEMPT'})

            smach.StateMachine.add('RECOVERY_SIMPLE_MOVE',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'RECOVERY_MANAGER',
                                                  'object_detected':'RECOVERY_MANAGER',
                                                  'preempted':'LEVEL_TWO_PREEMPTED',
                                                  'aborted':'LEVEL_TWO_ABORTED'})

            smach.StateMachine.add('CALCULATE_BEACON_ROTATION',
                                   CalculateBeaconRotation(self.tf_listener),
                                   transitions = {'next':'LOOK_AT_BEACON',
                                                  'aborted':'CREATE_MOVE_GOAL'})

            smach.StateMachine.add('LOOK_AT_BEACON',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'CALIBRATE_TO_BEACON',
                                                  'object_detected':'CALIBRATE_TO_BEACON',
                                                  'preempted':'LEVEL_TWO_PREEMPTED',
                                                  'aborted':'LEVEL_TWO_ABORTED'},
                                   remapping = {'simple_move':'beacon_turn',
                                                'stop_on_detection':'false'})

            smach.StateMachine.add('CALIBRATE_TO_BEACON',
                                   WaitForFlagState('false',
                                                    flag_trigger_value = 'true',
                                                    timeout = self.beacon_calibration_delay,
                                                    announcer = self.announcer,
                                                    start_message ='Calibrate ing to beacon.'),
                                   transitions = {'next':'FINISH_CALIBRATION',
                                                  'timeout':'FINISH_CALIBRATION',
                                                  'preempted':'LEVEL_TWO_PREEMPTED'})

            smach.StateMachine.add('FINISH_CALIBRATION',
                                   FinishCalibration(input_keys = ['map_calibration_requested'],
                                                     output_keys = ['map_calibration_requested'],
                                                     outcomes = ['first', 'recalibration']),
                                   transitions = {'first':'WEB_MANAGER',
                                                  'recalibration':'CREATE_MOVE_GOAL',})

            smach.StateMachine.add('LEVEL_TWO_PREEMPTED',
                                  LevelTwoPreempted(self.camera_enablers),
                                   transitions = {'recovery':'RECOVERY_MANAGER',
                                                  'calibrate':'CALCULATE_BEACON_ROTATION',
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

        self.time_remaining_pub = rospy.Publisher('minutes_remaining',
                                                  std_msg.Int16,
                                                  queue_size=1)
        rospy.Timer(rospy.Duration(2.0), self.publish_time_remaining)

        rospy.Service("start_recovery", samplereturn_srv.RecoveryParameters, self.start_recovery)

        #start a timer loop to check for localization updates
        rospy.Timer(rospy.Duration(2.0), self.localization_check)

        #start action servers and services
        sls.start()
        level_two_server.run_server()
        rospy.spin()
        sls.stop()

    def start_recovery(self, req):
        try:
            params = yaml.load(req.yaml)
            rospy.loginfo("START_RECOVERY received yaml params: {!s}".format(params))
            #get first namespace, and the first tuple entry (the param dict)
            #params = params[0][0]
        except Exception, exc:
            rospy.loginfo("START_RECOVERY failed to parse params: {!s}".format(exc))
            return False

        self.announcer.say("Enter ing recovery state.")
        self.state_machine.userdata.recovery_requested = True
        self.state_machine.userdata.recovery_parameters = params
        self.state_machine.request_preempt()
        return True

    def publish_time_remaining(self, event):
        if self.state_machine.is_running():
            return_time = self.state_machine.userdata.start_time + \
                          self.state_machine.userdata.return_time_offset + \
                          self.state_machine.userdata.pause_time_offset
            time_remaining = return_time - rospy.Time.now()
            self.time_remaining_pub.publish(int(time_remaining.to_sec()/60))

    def pause_state_update(self, msg):
        self.state_machine.userdata.paused = msg.data

        if self.state_machine.is_running():

            if (msg.data):  #if we were just paused
                self.state_machine.userdata.pause_start_time = rospy.Time.now()
            else:  #if we just received an unpause
                self.state_machine.userdata.pause_time_offset += \
                    rospy.Time.now() - self.state_machine.userdata.pause_start_time

            if (self.state_machine.userdata.pause_time_offset.to_sec() != 0 ):

                rospy.loginfo("LEVEL_TWO pause state change: start_time: {!s}, pause_time_offset: {:f}, return_offset_time: {:f}".format(
                    time.strftime("%H:%M:%S", time.localtime(self.state_machine.userdata.start_time.to_sec())),
                    self.state_machine.userdata.pause_time_offset.to_sec(),
                    self.state_machine.userdata.return_time_offset.to_sec()
                ))

    def sample_update(self, sample):
        #only update the detection message with samples if stop_on_sample is true
        if self.state_machine.userdata.report_sample:
            try:
                self.tf_listener.waitForTransform(self.odometry_frame,
                                                  sample.header.frame_id,
                                                  sample.header.stamp,
                                                  rospy.Duration(1.0))
                point_in_frame = self.tf_listener.transformPoint(self.odometry_frame, sample)
                sample.point = point_in_frame.point
                self.state_machine.userdata.detection_message = sample
            except tf.Exception:
                rospy.logwarn("LEVEL_TWO failed to transform search detection point %s->%s",
                              sample.header.frame_id, self.odometry_frame)

    def beacon_update(self, beacon_pose):
        #update the detection_message with the beacon if we want to stop when we see it
        rospy.logdebug("LEVEL_TWO report_beacon: {!s}, \
                       received beacon pose: {!s}".format(self.state_machine.userdata.report_beacon,
                                                          beacon_pose))
        self.last_beacon_point = geometry_msg.PointStamped(beacon_pose.header,
                                                     beacon_pose.pose.pose.position)
        if self.state_machine.userdata.report_beacon:
            self.state_machine.userdata.detection_message = self.last_beacon_point

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
                return

            correction_error = util.point_distance_2d(goal_point_odom,
                                                      saved_point_odom.point)

            if (np.abs(correction_error-self.last_correction_error) > 0.1):
                rospy.loginfo("LEVEL_TWO localization correction error: {:f}".format(correction_error))

            self.last_correction_error = correction_error

            if self.vfh_mover.get_state() in util.actionlib_working_states:

                if (correction_error > self.recalibrate_threshold):
                    rospy.loginfo("LEVEL_TWO handling large beacon correction.")
                    self.announcer.say("Large beacon correction.")
                    self.state_machine.userdata.beacon_point = self.last_beacon_point
                    self.state_machine.userdata.map_calibration_requested = True
                    self.state_machine.userdata.move_point_map = None
                    self.state_machine.request_preempt()

                elif (correction_error > self.replan_threshold):
                    rospy.loginfo("LEVEL_TWO replan for small beacon correction.")
                    self.announcer.say("Beacon correction.")
                    #update the VFH move goal
                    goal = deepcopy(self.state_machine.userdata.move_goal)
                    goal.target_pose.pose.position = saved_point_odom.point
                    self.state_machine.userdata.move_goal = goal

    def shutdown_cb(self):
        rospy.delete_param('~initial_behavior')
        self.state_machine.request_preempt()
        while self.state_machine.is_running():
            rospy.sleep(0.1)
        rospy.logwarn("EXECUTIVE LEVEL_TWO STATE MACHINE EXIT")
        rospy.sleep(0.2) #hideous hack delay to let action server get its final message out

#searches the globe
class StartLeveLTwo(smach.State):

    def __init__(self):

        smach.State.__init__(self,
                            input_keys=['paused',
                                        'initial_behavior',
                                        'spin_velocity'],
                            output_keys=['action_result',
                                        'recovery_parameters',
                                        'recovery_requested',
                                        'move_target',
                                        'start_time',
                                        'pause_start_time',
                                        'pause_time_offset'],
                            outcomes=['next']),
        
    def execute(self, userdata):

        result = samplereturn_msg.GeneralExecutiveResult()
        result.result_string = 'initialized'
        userdata.action_result = result
        userdata.move_target = None

        #load the initial behavior
        userdata.recovery_parameters = deepcopy(userdata.initial_behavior)
        userdata.recovery_requested = True

        userdata.pause_time_offset = rospy.Duration(0)
        userdata.start_time = rospy.Time.now()
        if userdata.paused:
            userdata.pause_start_time = rospy.Time.now()



        return 'next'

class WebManager(smach.State):
    def __init__(self, label, tf_listener, beacon_enable, search_enable, announcer):
        smach.State.__init__(self,
                             input_keys = ['outbound',
                                           'web_slices',
                                           'web_slice_indices',
                                           'default_velocity',
                                           'raster_velocity',
                                           'raster_active',
                                           'raster_points',
                                           'raster_step',
                                           'raster_tolerance',
                                           'chord_course_tolerance',
                                           'vfh_result',
                                           'manager_dict',
                                           'detection_message',
                                           'start_time',
                                           'return_time_offset',
                                           'pause_time_offset',
                                           'move_target',
                                           'world_fixed_frame'],
                             output_keys = ['outbound',
                                            'web_slice_indices',
                                            'raster_active',
                                            'raster_points',
                                            'move_target',
                                            'move_velocity',
                                            'course_tolerance',
                                            'allow_rotate_to_clear',
                                            'report_sample',
                                            'stop_on_detection',
                                            'active_manager'],
                             outcomes=['move',
                                       'get_raster',
                                       'sample_detected',
                                       'return_home',
                                       'preempted', 'aborted'])

        self.tf_listener = tf_listener
        self.beacon_enable = beacon_enable
        self.search_enable = search_enable
        self.announcer = announcer
        self.label = label
        self.last_web_move = None
        self.blocked_set = [VFHMoveResult.BLOCKED, VFHMoveResult.OFF_COURSE]

    def execute(self, userdata):
        #set the move manager key for the move mux
        userdata.active_manager = userdata.manager_dict[self.label]
        userdata.report_sample = True #always act on samples if this manager is working
        userdata.stop_on_detection = True

        return_time = userdata.start_time + userdata.pause_time_offset + userdata.return_time_offset

        if rospy.Time.now() > return_time:
            rospy.loginfo("WEB_MANAGER exceeded return_time")
            userdata.report_sample = False
            userdata.allow_rotate_to_clear = True
            userdata.stop_on_detection = False
            userdata.move_velocity = userdata.default_velocity
            return 'return_home'

        #this means the last move was preempted, we should finish it        
        if userdata.move_target is not None:
           return 'move'

        if len(userdata.web_slice_indices) == 0:
            rospy.loginfo("WEB_MANAGER finished last spoke")
            userdata.move_velocity = userdata.default_velocity
            return 'return_home'
        else:
            active_slice = userdata.web_slices[userdata.web_slice_indices[0]] 

        if isinstance(userdata.detection_message, samplereturn_msg.NamedPoint):
            #this is where we will go after pursuit
            userdata.move_target = self.last_web_move['point']       
            return 'sample_detected'

        if userdata.outbound:
            #this flag indicates it's time to make an outbound move
            end_point = get_polar_point(active_slice['start_angle'],
                                        active_slice['max_radius'],
                                        userdata.world_fixed_frame)
            next_move = {'point':end_point, 'radius':0, 'radial':True}
            userdata.outbound = False
            self.beacon_enable.publish(False)
            rospy.loginfo("WEB_MANAGER starting spoke: %.2f:" %(active_slice['start_angle']))
            rospy.loginfo("WEB_MANAGER remaining indices: {!s}".format(userdata.web_slice_indices))
            remaining_minutes = int((return_time - rospy.Time.now()).to_sec()/60)
            self.announcer.say("Start ing on spoke, yaw {!s}.  {:d} minutes left.  Beacon camera off.".format(
                                int(active_slice['start_angle']),
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
                    self.announcer.say("Return ing from raster on spoke, yaw {!s}. Beacon camera active. \
                                        Search cameras off.".format(int(active_slice['start_angle'])))
                    self.search_enable.publish(False)
                    self.beacon_enable.publish(True)
                    userdata.raster_active = False
                    userdata.web_slice_indices.popleft() #remove the current index
                    userdata.outbound = True #request the next spoke move
                else:
                #if not, do we happen to aleady be close enough to the next point
                #that planning to it is just a waste of time?
                    yaw, distance = util.get_robot_strafe(self.tf_listener,
                                                          next_move['point'])
                    if distance < userdata.raster_tolerance:
                        self.announcer.say("Already with in raster tolerance")
                        next_move = userdata.raster_points.popleft()

                return self.load_move(next_move, userdata)
            else:
                #time to start rastering, create the points, and set flag
                self.search_enable.publish(True)                
                self.announcer.say("Begin ing raster on spoke, yaw {!s}.  Search cameras active.".format(
                                    int(active_slice['start_angle'])))
                userdata.raster_active = True
                return 'get_raster'

        return 'aborted'

    def load_move(self, next_move, userdata):
        #use tighter course tolerance for chord moves (no point in getting into next chord)
        if next_move['radial']:
            userdata.allow_rotate_to_clear = True
            userdata.move_velocity = userdata.default_velocity
        else:
            userdata.course_tolerance = userdata.chord_course_tolerance
            userdata.move_velocity = userdata.raster_velocity
            userdata.allow_rotate_to_clear = False
        #load the target into move_point, and save the move
        #move_point is consumed by the general move_goal transformer,
        #and web_move is used to do retry_checks, etc.
        self.last_web_move = next_move
        userdata.move_target = next_move['point']

        return 'move'

class CreateRasterPoints(smach.State):
    def __init__(self, tf_listener):
        smach.State.__init__(self,
                             input_keys = ['web_slices',
                                           'web_slice_indices',
                                           'spoke_yaw',
                                           'spoke_hub_radius',
                                           'raster_step',
                                           'raster_offset',
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

            active_slice = userdata.web_slices[userdata.web_slice_indices[0]]
            current_yaw = active_slice['start_angle']
            next_yaw = active_slice['end_angle']


            rospy.loginfo("STARTING RADIUS, YAW, NEXT YAW: {!s}, {!s}, {!s}".format(radius,
                                                                                    current_yaw,
                                                                                    next_yaw))
            #generate no raster points if we didn't reach min radius
            if radius > active_slice['min_radius']:
                #generate one end_angle-inward-start_angle-inward set of points per loop
                while not rospy.is_shutdown():
                    #chord move to next yaw
                    point = get_polar_point(next_yaw, radius, userdata.world_fixed_frame)
                    raster_points.append({'point':point,'radius':radius,'radial':False})
                    radius -= userdata.raster_step
                    #return from end angle if flag is True
                    if radius < active_slice['min_radius'] and active_slice['return_on_end']:
                        break
                    #inward move on next yaw
                    point = get_polar_point(next_yaw, radius, userdata.world_fixed_frame)
                    raster_points.append({'point':point,'radius':radius,'radial':True})
                    #chord move back to current yaw
                    point = get_polar_point(current_yaw, radius, userdata.world_fixed_frame)
                    raster_points.append({'point':point,'radius':radius,'radial':False})
                    radius -= userdata.raster_step
                    #return from starting angle if specified
                    if radius < active_slice['min_radius'] and not active_slice['return_on_end']:
                        break
                    #inward move on current yaw
                    point = get_polar_point(current_yaw, radius, userdata.world_fixed_frame)
                    raster_points.append({'point':point,'radius':radius,'radial':True})

            final_yaw = next_yaw if active_slice['return_on_end'] else current_yaw
            point = get_polar_point(final_yaw, userdata.spoke_hub_radius, userdata.world_fixed_frame)
            raster_points.append({'point':point,'radius':userdata.spoke_hub_radius,'radial':True})

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
                                           'world_fixed_frame',
                                           'local_frame'],
                             output_keys = ['move_goal',
                                            'move_point_map',
                                            'course_tolerance'],
                             outcomes=['next',
                                       'preempted', 'aborted'])

        self.tf_listener = tf_listener

    def execute(self, userdata):

        odometry_frame = userdata.odometry_frame
        target = userdata.move_target
        orient_at_target = False

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

        #create goal, modify default fields as specified
        goal = VFHMoveGoal(target_pose = target_pose,
                           move_velocity = userdata.move_velocity,
                           spin_velocity = userdata.spin_velocity,
                           orient_at_target = orient_at_target)
        #special course_tolerance requested?
        if userdata.course_tolerance is not None:
            goal.course_tolerance = userdata.course_tolerance
        #if we are back here after starting a move in the blocked condition, try rotate to clear
        if userdata.vfh_result == VFHMoveResult.STARTED_BLOCKED:
            rospy.loginfo("LEVEL_TWO setting rotate_to_clear = True")
            goal.rotate_to_clear = True

        userdata.move_goal = goal

        #reset to default course tolerance after every move
        userdata.course_tolerance = None

        #store the point in map, to compare in the beacon update callback
        #unless the requested move was in local_frame
        if target.header.frame_id != userdata.local_frame:
            userdata.move_point_map = geometry_msg.PointStamped(map_pose.header,
                                                                map_pose.pose.position)

        return 'next'

#For retrying a move after blocked result, to ensure it wasn't a costmap glitch.
class RetryCheck(smach.State):

    def __init__(self, announcer):
        smach.State.__init__(self,
                             input_keys = ['retry_active',
                                           'blocked_retry_delay',
                                           'allow_rotate_to_clear',
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
            if userdata.retry_active or not userdata.allow_rotate_to_clear:
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
    def __init__(self, label, announcer, beacon_enable, search_enable, tf_listener):
        smach.State.__init__(self,
                             input_keys = ['recovery_parameters',
                                           'recovery_requested',
                                           'manager_dict',
                                           'move_target',
                                           'raster_active',
                                           'web_slice_indices',
                                           'web_slices',
                                           'return_time_offset',
                                           'detection_message',
                                           'world_fixed_frame',
                                           'local_frame',
                                           'spoke_hub_radius'],
                             output_keys = ['recovery_parameters',
                                            'recovery_requested',
                                            'active_manager',
                                            'web_slice_indices',
                                            'raster_active',
                                            'raster_points',
                                            'outbound',
                                            'return_time_offset',
                                            'move_target',
                                            'simple_move',
                                            'report_sample',
                                            'report_beacon',
                                            'detection_message',
                                            'stop_on_detection'],
                             outcomes=['move',
                                       'sample_detected',
                                       'simple_move',
                                       'return_manager',
                                       'web_manager',
                                       'wait_for_preempt',
                                       'aborted'])

        self.label = label
        self.beacon_enable = beacon_enable
        self.search_enable = search_enable
        self.announcer = announcer
        self.tf_listener = tf_listener
        self.exit_move = None

    def execute(self, userdata):

        if userdata.recovery_requested == True:
            #first time through
            params_dict = userdata.recovery_parameters #make this name shorter for the love of god
            rospy.loginfo("RECOVERY_MANAGER starting with params: {!s}".format(params_dict))
            
            #save these in case we are returning to the web manager
            self.exit_move = userdata.move_target
            userdata.detection_message = False
            userdata.recovery_requested = False
            userdata.report_beacon = False
            userdata.stop_on_detection = True
            
            if ('announcement') in params_dict:
                self.announcer.say(params_dict['announcement'])

            if ('pursue_samples' in params_dict) and params_dict['pursue_samples']:
                userdata.report_sample = True
            else:
                userdata.report_sample = False

            if 'beacon_enabled_on_entry' in params_dict:
                if params_dict['beacon_enabled_on_entry']:
                    self.announcer.say('Beacon camera active.')
                    self.beacon_enable.publish(True)
                else:
                    self.announcer.say('Beacon camera off.')
                    self.beacon_enable.publish(False)

            if 'search_enabled_on_entry' in params_dict:
                if params_dict['search_enabled_on_entry']:
                    self.announcer.say('Search cameras active.')
                    self.search_enable.publish(True)
                else:
                    self.announcer.say('Search cameras off.')
                    self.search_enable.publish(False)

            #modify return time
            if 'time_offset' in params_dict:
                time_offset = rospy.Duration(params_dict['time_offset']*60)
                userdata.return_time_offset += time_offset
            
            #prune requested web_slices
            if 'slices_to_remove' in params_dict:
                web_slices_to_remove = params_dict['slices_to_remove']
            else:
                web_slices_to_remove = 0
            if 'slices_to_add' in params_dict:
                slices_to_add = userdata.recovery_parameters['slices_to_add']
            else:
                slices_to_add = []
            if len(userdata.web_slice_indices) > 0:
                if web_slices_to_remove > 0:
                    userdata.raster_active = False
                    userdata.raster_points = deque()
                    userdata.outbound = True
                    point = get_polar_point(userdata.web_slices[userdata.web_slice_indices[0]]['end_angle'],
                                            userdata.spoke_hub_radius,
                                            userdata.world_fixed_frame)
                    self.exit_move = point
                    while web_slices_to_remove > 0:
                        #current spoke is leftmost index
                        if len(userdata.web_slice_indices) > 0:
                            userdata.web_slice_indices.popleft()
                        web_slices_to_remove -= 1
                elif len(slices_to_add) > 0:
                    #if we don't remove any spokes, we must add new slices after index 0
                    slices_to_add.insert(0, userdata.web_slice_indices.popleft())

            while (len(slices_to_add)>0):
                if slices_to_add[-1] < len(userdata.web_slices):
                        userdata.web_slice_indices.appendleft(slices_to_add.pop())                

        #set the move manager key for the move mux
        userdata.active_manager = userdata.manager_dict[self.label]

        #if a sample is detected, cancel recovery and exit to pursuit
        if isinstance(userdata.detection_message, samplereturn_msg.NamedPoint):
            userdata.active_manager = userdata.recovery_parameters['terminal_outcome']
            userdata.move_target = self.exit_move
            return 'sample_detected'

        if 'simple_moves' in userdata.recovery_parameters and \
         len(userdata.recovery_parameters['simple_moves']) > 0:
            simple_move = userdata.recovery_parameters['simple_moves'].pop(0)
            if 'distance' in simple_move:
                userdata.simple_move = SimpleMoveGoal(type=SimpleMoveGoal.STRAFE,
                                                      angle = np.radians(simple_move['angle']),
                                                      distance = simple_move['distance'],
                                                      velocity = simple_move['velocity'])
            else:
                userdata.simple_move = SimpleMoveGoal(type=SimpleMoveGoal.SPIN,
                                                      angle = np.radians(simple_move['angle']),
                                                      velocity = simple_move['velocity'])
            return 'simple_move'

        if 'moves' in userdata.recovery_parameters and \
         len(userdata.recovery_parameters['moves']) > 0:
            move = userdata.recovery_parameters['moves'].pop(0)
            base_header = std_msg.Header(0, rospy.Time(0), userdata.local_frame)
            base_pose_stamped = geometry_msg.PoseStamped(header = base_header)
            target_pose = util.pose_translate_by_yaw(base_pose_stamped,
                                                     move['distance'],
                                                     np.radians(move['rotate']))
            userdata.move_target = geometry_msg.PointStamped(target_pose.header,
                                                             target_pose.pose.position)
            return 'move'
        else:
            if 'beacon_enabled_on_exit' in userdata.recovery_parameters:
                if userdata.recovery_parameters['beacon_enabled_on_exit']:
                    self.announcer.say('Beacon camera active.')
                    self.beacon_enable.publish(True)
                else:
                    self.announcer.say('Beacon camera off.')
                    self.beacon_enable.publish(False)

            if 'search_enabled_on_exit' in userdata.recovery_parameters:
                if userdata.recovery_parameters['search_enabled_on_exit']:
                    self.announcer.say('Search cameras active.')
                    self.search_enable.publish(True)
                else:
                    self.announcer.say('Search cameras off.')
                    self.search_enable.publish(False)
            
            userdata.stop_on_detection = False
            #setting the active manager is for the return to web_manager
            #it's the only way I could think of to return to the previous goal
            #exiting to the return_manager will overwrite this stuff
            userdata.active_manager = userdata.recovery_parameters['terminal_outcome']
            userdata.move_target = self.exit_move
            return userdata.recovery_parameters['terminal_outcome']

class CalculateBeaconRotation(smach.State):

    def __init__(self, tf_listener):
        smach.State.__init__(self,
                             input_keys = ['beacon_point',
                                           'spin_velocity',
                                           'odometry_frame',
                                           'world_fixed_frame'],
                             output_keys = ['beacon_turn'],
                             outcomes = ['next',
                                         'aborted'])

        self.tf_listener = tf_listener

    def execute(self, userdata):
        point = userdata.beacon_point
        move = SimpleMoveGoal(type=SimpleMoveGoal.SPIN,
                              angle = 0.0,
                              velocity = userdata.spin_velocity)

        try:
            point_in_odom = self.tf_listener.transformPoint(userdata.odometry_frame,
                                                            point)
            point_in_odom.header.stamp = rospy.Time(0)
            yaw, dist = util.get_robot_strafe(self.tf_listener, point_in_odom)
            move.angle = yaw
        except tf.Exception, e:
            rospy.loginfo("LEVEL_TWO calibration rotation not calculated: {!s}".format(e))
            return 'aborted'

        userdata.beacon_turn = move
        return 'next'

class FinishCalibration(smach.State):

    def execute(self, userdata):
        if userdata.map_calibration_requested:
            userdata.map_calibration_requested = False
            return 'recalibration'
        else:
            return 'first'

class MoveMUX(smach.State):
    def __init__(self, manager_list):
        smach.State.__init__(self,
                             input_keys = ['active_manager'],
                             output_keys = ['retry_active',
                                            'move_target',
                                            'move_point_map'],
                             outcomes = manager_list)

    def execute(self, userdata):
        #do not clear move target on preempt
        userdata.retry_active = False
        userdata.move_point_map = None
        if not self.preempt_requested():                
            userdata.move_target = None        
            return userdata.active_manager
        else:
            return 'preempted'

class LevelTwoPreempted(smach.State):
    def __init__(self, camera_enablers):
        smach.State.__init__(self,
                             input_keys = ['recovery_requested',
                                           'map_calibration_requested'],
                             output_keys = ['move_point_map'],
                             outcomes=['recovery', 'calibrate', 'exit'])

        self.camera_enablers = camera_enablers

    def execute(self, userdata):

        userdata.move_point_map = None

        if userdata.recovery_requested:
            return 'recovery'
        elif userdata.map_calibration_requested:
            return 'calibrate'

        #disable all search/beacon cameras
        for publisher in self.camera_enablers:
            publisher.publish(False)

        return 'exit'

class LevelTwoAborted(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next'])

    def execute(self, userdata):

        return 'next'

def get_polar_point(angle, distance, world_frame):
    yaw = util.unwind(np.radians(angle))
    header = std_msg.Header(0, rospy.Time(0), world_frame)
    pos = geometry_msg.Point(distance*math.cos(yaw),
                             distance*math.sin(yaw), 0)
    point = geometry_msg.PointStamped(header, pos)
    return point
