#!/usr/bin/env python
import math
import collections
import threading
from copy import deepcopy
import numpy as np

import smach
import smach_ros
import rospy
import actionlib
import tf

import std_msgs.msg as std_msg
import actionlib_msgs.msg as action_msg
import visual_servo_msgs.msg as visual_servo_msg
import samplereturn_msgs.msg as samplereturn_msg
import platform_motion_msgs.msg as platform_msg
import platform_motion_msgs.srv as platform_srv
import manipulator_msgs.msg as manipulator_msg
import move_base_msgs.msg as move_base_msg
import geometry_msgs.msg as geometry_msg
import samplereturn_msgs.srv as samplereturn_srv
import visualization_msgs.msg as vis_msg

import samplereturn.util as util
import motion_planning.simple_motion as simple_motion
from motion_planning.simple_motion import TimeoutException
from samplereturn_msgs.msg import SimpleMoveGoal
from sun_pointing.msg import ComputeAngleAction, ComputeAngleGoal

from executive.executive_states import WaitForFlagState
from executive.executive_states import AnnounceState
from executive.executive_states import SelectMotionMode
from executive.executive_states import ServoController
from executive.executive_states import ExecuteSimpleMove
from executive.executive_states import ExecuteVFHMove
from executive.executive_states import MoveToPoints

class PursueSample(object):
    
    def __init__(self):
        
        rospy.on_shutdown(self.shutdown_cb)
        self.node_params = util.get_node_params()
        self.tf_listener = tf.TransformListener()

        self.world_fixed_frame = rospy.get_param("world_fixed_frame", "map")
        self.odometry_frame = rospy.get_param("odometry_frame", "odom")
        
        #interfaces
        self.announcer = util.AnnouncerInterface("audio_search")
        self.result_pub = rospy.Publisher('pursuit_result', samplereturn_msg.PursuitResult, queue_size=1)
        self.light_pub = rospy.Publisher('search_lights', std_msg.Bool, queue_size=1)
        self.CAN_interface = util.CANInterface()
        
        #get mover action clients
        self.vfh_mover = actionlib.SimpleActionClient("vfh_move",
                                                       samplereturn_msg.VFHMoveAction)        
        self.simple_mover = actionlib.SimpleActionClient("simple_move",
                                                       samplereturn_msg.SimpleMoveAction)
        
       
        #for this state machine, there is no preempt path.  It either finshes successfully and
        #reports success on the PursuitResult topic, or in case of any interupption or failure, it
        #exits through the abort path, reporting failure on the topic.
        self.state_machine = smach.StateMachine(
                outcomes=['complete', 'preempted', 'aborted'],
                input_keys = ['action_goal'],
                output_keys = ['action_result'])
        
        #tf frames
        self.state_machine.userdata.odometry_frame = self.odometry_frame
        self.state_machine.userdata.world_fixed_frame = self.world_fixed_frame
        
        #userdata that subscribers fiddle with
        self.state_machine.userdata.paused = False        
        self.state_machine.userdata.detected_sample = None
        self.state_machine.userdata.target_sample = None
        self.state_machine.userdata.pursuit_goal = None
            #latched sample is the sample information returned from the servo controller, which
            #is hopefully the identification made by the manipulator detector
        self.state_machine.latched_sample = None
        
        #pursuit params
        self.max_pursuit_error = self.node_params.max_pursuit_error
        self.min_pursuit_distance = self.node_params.min_pursuit_distance
        self.state_machine.userdata.pursuit_velocity = self.node_params.pursuit_velocity
        self.state_machine.userdata.spin_velocity = self.node_params.spin_velocity
        self.state_machine.userdata.final_pursuit_step = self.node_params.final_pursuit_step
        self.state_machine.userdata.min_pursuit_distance = self.node_params.min_pursuit_distance
        self.state_machine.userdata.max_pursuit_error = self.node_params.max_pursuit_error        
        self.state_machine.userdata.sample_yaw_tolerance = np.pi/4
        self.sample_obstacle_check_width = self.node_params.sample_obstacle_check_width
            #latched filter id is the id of the hypothesis that triggered this pursuit
            #if it ever disappears or becomes impossible to get to, etc, publish failure
        self.state_machine.userdata.latched_filter_id = None


        
        #stop function check flags        
        self.state_machine.userdata.check_pursuit_distance = False
        self.state_machine.userdata.stop_on_detection = False

        #area search and servo settings, search_count limits the number of times we can enter the
        #sample_lost search pattern
        self.state_machine.userdata.search_velocity = self.node_params.search_velocity
        self.state_machine.userdata.square_search_size = self.node_params.square_search_size
        self.state_machine.userdata.settle_time = rospy.Duration(self.node_params.settle_time)
        self.state_machine.userdata.manipulator_search_angle = math.pi/4
        self.state_machine.userdata.search_count = 0
        self.state_machine.userdata.search_try_limit = 2
        self.state_machine.userdata.grab_count = 0
        self.state_machine.userdata.grab_count_limit = 2
        self.state_machine.userdata.manipulator_correction = self.node_params.manipulator_correction
        self.state_machine.userdata.servo_params = self.node_params.servo_params
        
        #total bullshit for bins
        self.state_machine.userdata.available_small_bins = [1,2,3,8,9,10]
        self.state_machine.userdata.available_big_bins = [4,5,6,7]
        self.state_machine.userdata.big_sample_ids = [4,6,7,10]
        self.state_machine.userdata.active_bin_id = 0
        
        #use these as booleans in remaps
        self.state_machine.userdata.true = True
        self.state_machine.userdata.false = False

        #motion mode stuff
        planner_mode = self.node_params.planner_mode
        MODE_PLANNER = getattr(platform_srv.SelectMotionModeRequest, planner_mode)
        MODE_SERVO = platform_srv.SelectMotionModeRequest.MODE_SERVO            

        with self.state_machine:
            
            smach.StateMachine.add('START_SAMPLE_PURSUIT',
                                   StartSamplePursuit(self.tf_listener),
                                   transitions = {'next':'ANNOUNCE_SAMPLE_PURSUIT'})
            
            smach.StateMachine.add('ANNOUNCE_SAMPLE_PURSUIT',
                                   AnnounceState(self.announcer,
                                                 'Sample detected.  Pursue ing'),
                                   transitions = {'next':'APPROACH_SAMPLE'})
 
            smach.StateMachine.add('APPROACH_SAMPLE',
                                   ExecuteVFHMove(self.vfh_mover),
                                   transitions = {'complete':'ANNOUNCE_MANIPULATOR_APPROACH',
                                                  'blocked':'PUBLISH_FAILURE',
                                                  'started_blocked':'PUBLISH_FAILURE',
                                                  'missed_target':'ANNOUNCE_MANIPULATOR_APPROACH',
                                                  'off_course':'PUBLISH_FAILURE',
                                                  'object_detected':'PUBLISH_FAILURE',
                                                  'preempted':'PUBLISH_FAILURE',
                                                  'aborted':'PUBLISH_FAILURE'},
                                   remapping = {'move_goal':'pursuit_goal'})

            '''
            @smach.cb_interface(outcomes=['verified','not_verified'])
            def sample_verify_resp(userdata, response):
                if response.verified:
                    return 'verified'
                else:
                    return 'not_verified'
            
            @smach.cb_interface(input_keys=['target_sample'])
            def sample_verify_req(userdata, request):
                return samplereturn_srv.VerifyRequest(target = userdata.target_sample)
            
            smach.StateMachine.add('SAMPLE_VERIFY',
                smach_ros.ServiceState('close_range_verify', samplereturn_srv.Verify,
                request_cb = sample_verify_req,
                response_cb = sample_verify_resp),
                transitions = {'verified':'ANNOUNCE_VERIFIED',
                               'not_verified':'ANNOUNCE_NOT_VERIFIED',
                               'succeeded':'PUBLISH_FAILURE', #means cb error I think
                               'aborted':'PUBLISH_FAILURE'})

            smach.StateMachine.add('ANNOUNCE_VERIFIED',
                                   AnnounceState(self.announcer,
                                                 'Sample verified.  Prepare ing final approach.'),
                                   transitions = {'next':'CALCULATE_MANIPULATOR_APPROACH'})    

            smach.StateMachine.add('ANNOUNCE_NOT_VERIFIED',
                                   AnnounceState(self.announcer,
                                                 'Sample not verified.'),
                                   transitions = {'next':'PUBLISH_FAILURE'})    
            '''

            smach.StateMachine.add('ANNOUNCE_MANIPULATOR_APPROACH',
                                           AnnounceState(self.announcer,
                                                         'Prepare ing final approach.'),
                                           transitions = {'next':'CALCULATE_MANIPULATOR_APPROACH'})    

             #calculate the final strafe move to the sample, this gets us the yaw for the obstacle check
            smach.StateMachine.add('CALCULATE_MANIPULATOR_APPROACH',
                                   CalculateManipulatorApproach(self.tf_listener),
                                   transitions = {'move':'OBSTACLE_CHECK',
                                                  'retry_approach':'ANNOUNCE_RETRY_APPROACH',
                                                  'point_lost':'ANNOUNCE_POINT_LOST',
                                                  'aborted':'PURSUE_SAMPLE_ABORTED'})
            
            smach.StateMachine.add('ANNOUNCE_RETRY_APPROACH',
                                   AnnounceState(self.announcer,
                                                 'Sample not in tolerance, retry ing approach'),
                                   transitions = {'next':'APPROACH_SAMPLE'})            

            @smach.cb_interface(outcomes=['clear','blocked'])
            def obstacle_check_resp(userdata, response):
                if response.obstacle:
                    return 'blocked'
                else:
                    return 'clear'
            
            #request_cb for obstacle check, gets the yaw from userdata    
            @smach.cb_interface(input_keys=['target_yaw', 'target_distance'])
            def obstacle_check_req(userdata, request):
                check_distance = self.sample_obstacle_check_width/2. + userdata.target_distance
                return samplereturn_srv.ObstacleCheckRequest(yaw = userdata.target_yaw,
                                                             width = self.sample_obstacle_check_width,
                                                             distance = check_distance)
            
            smach.StateMachine.add('OBSTACLE_CHECK',
                smach_ros.ServiceState('obstacle_check', samplereturn_srv.ObstacleCheck,
                request_cb = obstacle_check_req,
                response_cb = obstacle_check_resp),
                transitions = {'blocked':'ANNOUNCE_BLOCKED',
                               'clear':'ANNOUNCE_CLEAR',
                               'succeeded':'PUBLISH_FAILURE', #means cb error I think
                               'aborted':'PUBLISH_FAILURE'})
    
            smach.StateMachine.add('ANNOUNCE_BLOCKED',
                                   AnnounceState(self.announcer,
                                                 'Obstacles in sample area.'),
                                   transitions = {'next':'PUBLISH_FAILURE'})
            
            smach.StateMachine.add('ANNOUNCE_CLEAR',
                                   AnnounceState(self.announcer,
                                                 'Area clear. Begin ing approach.'),
                                   transitions = {'next':'ENABLE_MANIPULATOR_DETECTOR'})


            smach.StateMachine.add('ENABLE_MANIPULATOR_DETECTOR',
                                    smach_ros.ServiceState('enable_manipulator_detector',
                                                            samplereturn_srv.Enable,
                                                            request = samplereturn_srv.EnableRequest(True)),
                                     transitions = {'succeeded':'MANIPULATOR_APPROACH_MOVE',
                                                    'aborted':'PUBLISH_FAILURE'})
            
            smach.StateMachine.add('MANIPULATOR_APPROACH_MOVE',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'MANIPULATOR_FINAL_MOVE',
                                                  'object_detected':'VISUAL_SERVO',
                                                  'aborted':'PUBLISH_FAILURE'},
                                   remapping = {'detection_message':'detected_sample',
                                                'stop_on_detection':'true'})
            
            smach.StateMachine.add('MANIPULATOR_FINAL_MOVE',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'ANNOUNCE_SUN_POINTING',
                                                  'object_detected':'VISUAL_SERVO',
                                                  'aborted':'PUBLISH_FAILURE'},
                                   remapping = {'simple_move':'final_move',
                                                'detection_message':'detected_sample',
                                                'stop_on_detection':'true'})
            
            smach.StateMachine.add('ANNOUNCE_SUN_POINTING',
                                   AnnounceState(self.announcer,
                                                 "No sample detected, rotate ing to optimal light condition."),
                                   transitions = {'next':'SUN_POINTING_ROTATION'})            
            
            smach.StateMachine.add('SUN_POINTING_ROTATION',
                                   smach_ros.SimpleActionState('sun_pointing',
                                   ComputeAngleAction,
                                   goal = ComputeAngleGoal(True)),
                                   transitions = {'succeeded':'HANDLE_SEARCH',
                                                  'preempted':'PUBLISH_FAILURE',
                                                  'aborted':'PUBLISH_FAILURE'})            
            
            smach.StateMachine.add('HANDLE_SEARCH',
                                   HandleSearch(self.tf_listener, self.announcer),
                                   transitions = {'sample_search':'HANDLE_SEARCH_MOVES',
                                                  'no_sample':'PUBLISH_FAILURE'})

            smach.StateMachine.add('HANDLE_SEARCH_MOVES',
                                   MoveToPoints(self.tf_listener),
                                   transitions = {'next_point':'SEARCH_MOVE',
                                                  'complete':'ANNOUNCE_SEARCH_FAILURE',
                                                  'aborted':'PUBLISH_FAILURE'},
                                   remapping = {'velocity':'search_velocity'})
   
            smach.StateMachine.add('SEARCH_MOVE',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'HANDLE_SEARCH_MOVES',
                                                  'object_detected':'VISUAL_SERVO',
                                                  'aborted':'PUBLISH_FAILURE'},
                                   remapping = {'detection_message':'detected_sample',
                                                'stop_on_detection':'true'})

            smach.StateMachine.add('ANNOUNCE_SEARCH_FAILURE',
                                   AnnounceState(self.announcer,
                                                 "Search complete, no sample found"),
                                   transitions = {'next':'PUBLISH_FAILURE'})

            #calculate the strafe move to the sample
            smach.StateMachine.add('VISUAL_SERVO',
                                   ServoController(self.tf_listener, self.announcer),
                                   transitions = {'move':'SERVO_MOVE',
                                                  'complete':'ANNOUNCE_GRAB',
                                                  'point_lost':'HANDLE_SEARCH',
                                                  'aborted':'PUBLISH_FAILURE'})

            smach.StateMachine.add('SERVO_MOVE',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'VISUAL_SERVO',
                                                  'object_detected':'VISUAL_SERVO',
                                                  'aborted':'PUBLISH_FAILURE'},
                                   remapping = {'stop_on_detection':'false'})

            smach.StateMachine.add('ANNOUNCE_GRAB',
                                   AnnounceState(self.announcer,
                                                 "Deploy ing gripper"),
                                   transitions = {'next':'GRAB_SAMPLE'})

            @smach.cb_interface(input_keys=['latched_sample',
                                            'available_small_bins',
                                            'available_big_bins',
                                            'big_sample_ids'],
                                output_keys=['available_small_bins',
                                             'available_big_bins',
                                             'active_bin_id'])
            def grab_goal_cb(userdata, request):
                goal = manipulator_msg.ManipulatorGoal()
                goal.type = goal.GRAB
                goal.wrist_angle = userdata.latched_sample.grip_angle                    
                #decide to put in next small or big bin
                if (userdata.latched_sample.sample_id in userdata.big_sample_ids):
                    goal.target_bin = userdata.available_big_bins[0]
                    userdata.active_bin_id = goal.target_bin
                else:
                    goal.target_bin = userdata.available_small_bins[0]
                    userdata.active_bin_id = goal.target_bin
                goal.grip_torque = 0.7
                rospy.loginfo("PURSUE_SAMPLE target_bin: {!s}, sample_id: {!s},\
                              goal: {!s}".format(goal.target_bin,
                                          userdata.latched_sample.sample_id,
                                          goal))
                rospy.loginfo("PURSUE_SAMPLE available_small_bins: {!s}".format(userdata.available_small_bins))
                rospy.loginfo("PURSUE_SAMPLE available_big_bins: {!s}".format(userdata.available_big_bins))
                return goal
    
            #if Steve pauses the robot during this action, it returns preempted,
            #return to visual servo after pause, in case robot moved
            smach.StateMachine.add('GRAB_SAMPLE',
                                   smach_ros.SimpleActionState('manipulator_action',
                                   manipulator_msg.ManipulatorAction,
                                   goal_cb = grab_goal_cb),
                                   transitions = {'succeeded':'CONFIRM_SAMPLE_ACQUIRED',
                                                  'preempted':'CHECK_PAUSE_STATE',
                                                  'aborted':'PUBLISH_FAILURE'})

            smach.StateMachine.add('CONFIRM_SAMPLE_ACQUIRED',
                                   ConfirmSampleAcquired(self.announcer, self.result_pub),
                                   transitions = {'sample_gone':'DISABLE_MANIPULATOR_DETECTOR',
                                                  'sample_present':'VISUAL_SERVO',
                                                  'aborted':'PUBLISH_FAILURE'})

            #if the grab action is preempted by shutdown (or other non-pause reason),
            #exit pursue sample.  If paused, wait for unpause
            smach.StateMachine.add('CHECK_PAUSE_STATE',
                                   WaitForFlagState('paused',
                                                    flag_trigger_value = True),
                                   transitions = {'next':'WAIT_FOR_UNPAUSE',
                                                  'timeout':'PUBLISH_FAILURE',
                                                  'preempted':'PUBLISH_FAILURE'})
            
            smach.StateMachine.add('WAIT_FOR_UNPAUSE',
                                   WaitForFlagState('paused',
                                                    flag_trigger_value = False,
                                                    timeout = 20,
                                                    announcer = self.announcer,
                                                    start_message ='Pursuit paused, waiting for un pause'),
                                   transitions = {'next':'VISUAL_SERVO',
                                                  'timeout':'WAIT_FOR_UNPAUSE',
                                                  'preempted':'PUBLISH_FAILURE'})

            smach.StateMachine.add('ANNOUNCE_POINT_LOST',
                                   AnnounceState(self.announcer,
                                                 "Sample lost."),
                                   transitions = {'next':'PUBLISH_FAILURE'})

            smach.StateMachine.add('PUBLISH_FAILURE',
                                   PublishFailure(self.result_pub),
                                   transitions = {'next':'DISABLE_MANIPULATOR_DETECTOR'})

            #beginning of clean exit path
            smach.StateMachine.add('DISABLE_MANIPULATOR_DETECTOR',
                                    smach_ros.ServiceState('enable_manipulator_detector',
                                                            samplereturn_srv.Enable,
                                                            request = samplereturn_srv.EnableRequest(False)),
                                     transitions = {'succeeded':'ANNOUNCE_CONTINUE',
                                                    'aborted':'ANNOUNCE_CONTINUE'})

            smach.StateMachine.add('ANNOUNCE_CONTINUE',
                                   AnnounceState(self.announcer,
                                                 "Exit ing pursuit."),
                                   transitions = {'next':'complete'})

            smach.StateMachine.add('ANNOUNCE_RETURN',
                                   AnnounceState(self.announcer,
                                                 "Exit ing pursuit."),
                                   transitions = {'next':'complete'})

            smach.StateMachine.add('PURSUE_SAMPLE_ABORTED',
                                   PursueSampleAborted(self.result_pub),
                                   transitions = {'next':'aborted'})

        #action server wrapper    
        pursue_sample_server = smach_ros.ActionServerWrapper(
            'pursue_sample', samplereturn_msg.GeneralExecutiveAction,
            wrapped_container = self.state_machine,
            succeeded_outcomes = ['complete'],
            preempted_outcomes = ['preempted'],
            aborted_outcomes = ['aborted'],
            goal_key = 'action_goal',
            result_key = 'action_result')
        
        #introspection server
        sls = smach_ros.IntrospectionServer('smach_grab_introspection',
                                            self.state_machine,
                                            '/START_SAMPLE_PURSUIT')
      
        #subscribers, need to go after state_machine
        rospy.Subscriber('detected_sample_search',
                        samplereturn_msg.NamedPoint,
                        self.sample_detection_search)
        
        rospy.Subscriber('detected_sample_manipulator',
                        samplereturn_msg.NamedPoint,
                        self.sample_detection_manipulator)
        
         
        rospy.Subscriber("pause_state", std_msg.Bool, self.pause_state_update)

        #start action servers and services
        sls.start()
        pursue_sample_server.run_server()
        rospy.spin()
        sls.stop()

    def sample_detection_search(self, sample):
        #don't bother with this unless a latched filter id is active,
        #and it matches the one we have been tasked with pursuing
        latched_filter_id = self.state_machine.userdata.latched_filter_id
        if (latched_filter_id is not None) and (latched_filter_id == sample.filter_id):    

            point, pose = calculate_pursuit(self.tf_listener,
                                            sample,
                                            self.min_pursuit_distance,
                                            self.odometry_frame)
            
            self.state_machine.userdata.target_sample = point
                                   
            #if the pursuit_goal is None, we are not actively in pursuit
            if self.state_machine.userdata.pursuit_goal is not None:
                #if the desired pursuit pose changes too much, update the state_machine's pose
                pursuit_error = util.pose_distance_2d(self.state_machine.userdata.pursuit_goal.target_pose,
                                                      pose)
                if (pursuit_error > self.max_pursuit_error):
                    self.state_machine.pursuit_pose = pose
                    goal = samplereturn_msg.VFHMoveGoal(target_pose = pose,
                                                        move_velocity = self.state_machine.userdata.pursuit_velocity,
                                                        spin_velocity = self.state_machine.userdata.spin_velocity,
                                                        orient_at_target = True)
                    self.state_machine.userdata.pursuit_goal = goal
         
    def sample_detection_manipulator(self, sample):
        self.state_machine.userdata.detected_sample = sample
 
    def pause_state_update(self, msg):
        self.state_machine.userdata.paused = msg.data
            
    def shutdown_cb(self):
        self.state_machine.request_preempt()
        while self.state_machine.is_running():
            rospy.sleep(0.1)
        rospy.sleep(0.2) #hideous hack delay to let action server get its final message out
        rospy.logwarn("EXECUTIVE PURSUE_SAMPLE STATE MACHINE EXIT")
    
#searches the globe   
class StartSamplePursuit(smach.State):
    def __init__(self, tf_listener):
        smach.State.__init__(self,
                             outcomes=['next'],
                             input_keys=['action_goal',
                                         'pursuit_velocity',
                                         'spin_velocity',
                                         'min_pursuit_distance',
                                         'odometry_frame'],
                             output_keys=['latched_filter_id',
                                          'return_goal',
                                          'pursuit_goal',
                                          'action_result',
                                          'search_count',
                                          'grab_count'])
        
        self.tf_listener = tf_listener
    
    def execute(self, userdata):
        
        result = samplereturn_msg.GeneralExecutiveResult()
        result.result_string = 'approach failed'
        result.result_int = samplereturn_msg.NamedPoint.NONE
        userdata.action_result = result
        #default velocity for all moves is in simple_mover!
        #initial approach pursuit done at pursuit_velocity
        userdata.search_count = 0
        userdata.grab_count = 0

        rospy.loginfo("SAMPLE_PURSUIT input_point: %s" % (userdata.action_goal.input_point))        
        
        #store the filter_id on entry, this instance of pursuit will only try to pick up this hypothesis
        userdata.latched_filter_id = userdata.action_goal.input_point.filter_id
        
        point, pose = calculate_pursuit(self.tf_listener,
                                        userdata.action_goal.input_point,
                                        userdata.min_pursuit_distance,
                                        userdata.odometry_frame)

        #this is the initial vfh goal pose
        goal = samplereturn_msg.VFHMoveGoal(target_pose = pose,
                                            move_velocity = userdata.pursuit_velocity,
                                            spin_velocity = userdata.spin_velocity,
                                            orient_at_target = True)
        userdata.pursuit_goal = goal
       
        #create return destination
        current_pose = util.get_current_robot_pose(self.tf_listener,
                                                   userdata.odometry_frame)
        current_pose.header.stamp = rospy.Time(0)
        goal = samplereturn_msg.VFHMoveGoal(target_pose = current_pose,
                                            move_velocity = userdata.pursuit_velocity,
                                            spin_velocity = userdata.spin_velocity,
                                            orient_at_target = False)
        userdata.return_goal = goal        
                
        return 'next'

#waits for the chassis to settle and then get a good image of the maybe-sample,  then load
#a set of simple approach moves to get the manipulator cameras directly over the target
class CalculateManipulatorApproach(smach.State):
    def __init__(self, tf_listener):
        smach.State.__init__(self,
                             outcomes=['move', 'retry_approach', 'complete', 'point_lost', 'aborted'],
                             input_keys=['target_sample',
                                         'settle_time',
                                         'sample_yaw_tolerance',
                                         'final_pursuit_step',
                                         'pursuit_velocity',
                                         'search_velocity',
                                         'odometry_frame'],
                             output_keys=['simple_move',
                                          'final_move',
                                          'target_sample',
                                          'target_yaw',
                                          'target_distance',
                                          'detected_sample'])
        
        self.tf_listener = tf_listener
        
    def execute(self, userdata):

        #wait the settle time, then see if the sample is still there
        #timeout for this is also settle time
        userdata.target_sample = None
        rospy.sleep(userdata.settle_time)
        userdata.target_sample = None
        start_time = rospy.Time.now()
        while not rospy.core.is_shutdown_requested():
            rospy.sleep(0.1)
            if userdata.target_sample is not None:
                break #sample is still there, approach
            if (rospy.Time.now() - start_time) > userdata.settle_time:
                return 'point_lost' #sample disappeared, give up
        
        try:
            yaw, distance = util.get_robot_strafe(self.tf_listener, userdata.target_sample)
            robot_yaw = util.get_current_robot_yaw(self.tf_listener, userdata.odometry_frame)
        except tf.Exception:
            rospy.logwarn("PURSUE_SAMPLE failed to get base_link -> %s transform in 1.0 seconds", sample_frame)
            return 'aborted'
        rospy.loginfo("MANIPULATOR_APPROACH calculated with distance: {:f}, yaw: {:f} ".format(distance, yaw))
        #if, for some bizarre reason, the sample is now in a way different spot, re-approach
        if np.abs(yaw) > userdata.sample_yaw_tolerance:
            return 'retry_approach'
        #load sample distance and yaw for obstacle check      
        userdata.target_yaw = robot_yaw + yaw
        userdata.target_distance = distance
        #clear detected_sample prior to manipulator approach
        userdata.detected_sample = None
        distance -= userdata.final_pursuit_step
        userdata.simple_move = SimpleMoveGoal(type=SimpleMoveGoal.STRAFE,
                                              angle = yaw,
                                              distance = distance,
                                              velocity = userdata.pursuit_velocity)
        userdata.final_move = SimpleMoveGoal(type=SimpleMoveGoal.STRAFE,
                                             angle = yaw,
                                             distance = userdata.final_pursuit_step,
                                             velocity = userdata.search_velocity)
        return 'move'

class HandleSearch(smach.State):
    def __init__(self, tf_listener, announcer):
        smach.State.__init__(self,
                             outcomes=['sample_search', 'no_sample'],
                             input_keys=['square_search_size',
                                         'search_count',
                                         'search_try_limit',
                                         'odometry_frame',
                                         'detected_sample'],
                             output_keys=['point_list',
                                          'search_count'])
    
        self.tf_listener = tf_listener
        self.announcer = announcer

    def execute(self, userdata):

        if userdata.search_count >= userdata.search_try_limit:
            self.announcer.say("Search limit reached")
            return 'no_sample'

        point_list = collections.deque([])
        square_step = userdata.square_search_size

        try:
            start_pose = util.get_current_robot_pose(self.tf_listener,
                    userdata.odometry_frame)
            next_pose = util.translate_base_link(self.tf_listener, start_pose,
                    square_step, 0, userdata.odometry_frame)
            point_list.append(next_pose.pose.position)
            next_pose = util.translate_base_link(self.tf_listener, start_pose,
                    square_step, square_step, userdata.odometry_frame)
            point_list.append(next_pose.pose.position)
            next_pose = util.translate_base_link(self.tf_listener, start_pose,
                    -square_step, square_step, userdata.odometry_frame)
            point_list.append(next_pose.pose.position)
            next_pose = util.translate_base_link(self.tf_listener, start_pose,
                    -square_step, -square_step, userdata.odometry_frame)
            point_list.append(next_pose.pose.position)
            next_pose = util.translate_base_link(self.tf_listener, start_pose,
                    square_step, -square_step, userdata.odometry_frame)
            point_list.append(next_pose.pose.position)
            userdata.point_list = point_list
            self.announcer.say("Search ing area")           
            userdata.search_count += 1
            return 'sample_search'
            
        except tf.Exception, e:
            rospy.logwarn("PURSUE_SAMPLE failed to transform %s-%s in GetSearchMoves: %s",
                    'base_link', userdata.odometry_frame, e)
            return 'aborted'


class ConfirmSampleAcquired(smach.State):
    def __init__(self, announcer, result_pub):
        smach.State.__init__(self,
                             outcomes=['sample_gone',
                                       'sample_present',
                                       'aborted'],
                             input_keys=['latched_sample',
                                         'latched_filter_id',
                                         'detected_sample',
                                         'action_result',
                                         'grab_count',
                                         'grab_count_limit',
                                         'active_bin_id',
                                         'available_small_bins',
                                         'available_big_bins'],
                             output_keys=['detected_sample',
                                          'grab_count',
                                          'action_result',
                                          'pursuit_goal',
                                          'available_small_bins',
                                          'available_big_bins'])

        self.announcer = announcer
        self.result_pub = result_pub
    
    def execute(self, userdata):
        
        userdata.grab_count += 1
        
        #wait for 1 second, see if sample is present in view
        userdata.detected_sample = None
        rospy.sleep(1.0)
        if userdata.detected_sample is None:
            #this is the path of great success, notify filter nodes about success
            #and return the acquired sample ID in case the calling executive needs it
            self.announcer.say("Sample acquired")
            self.result_pub.publish(samplereturn_msg.PursuitResult(id = userdata.latched_filter_id,
                                                                   success = True))
            userdata.action_result.result_string = ('sample acquired')
            userdata.action_result.result_int = userdata.latched_sample.sample_id
            #de-allocate the bin if it isn't the last one in the list
            rospy.loginfo("PURSUE_SAMPLE placed sample in bin: {!s}".format(userdata.active_bin_id))
            if (userdata.active_bin_id in userdata.available_small_bins) \
            and (len(userdata.available_small_bins) > 1):
                used_bin = userdata.available_small_bins.pop(0)
                rospy.loginfo("PURSUE_SAMPLE removing small_bin: {!s}".format(used_bin))
            if (userdata.active_bin_id in userdata.available_big_bins) \
            and (len(userdata.available_big_bins) > 1):
                used_bin = userdata.available_big_bins.pop(0)
                rospy.loginfo("PURSUE_SAMPLE removing big_bin: {!s}".format(used_bin))
            userdata.pursuit_goal = None #finally, clear the pursuit_goal for next time
            return 'sample_gone'
        else:
            if userdata.detected_sample.sample_id == userdata.latched_sample.sample_id:
                if userdata.grab_count > userdata.grab_count_limit:
                    self.announcer.say("Sample acquisition failed.")
                    return 'aborted'
                else:
                    self.announcer.say("Sample acquisition failed. Retry ing")
                    return 'sample_present'
            else:
                self.announcer.say("New sample in view, confuse ing")
                return 'sample_gone'

class SearchLightSwitch(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['next'],
                             input_keys =['on'])
        
    def execute(self, userdata):
        pass

class PublishFailure(smach.State):
    def __init__(self, result_pub):
        smach.State.__init__(self,
                             outcomes=['next'],
                             input_keys = ['latched_filter_id'],
                             output_keys=['detected_sample',
                                          'pursuit_goal'])
        self.result_pub = result_pub
    def execute(self, userdata):
        userdata.detected_sample = None
        userdata.pursuit_goal = None #finally, clear the pursuit_goal for next time
        self.result_pub.publish(samplereturn_msg.PursuitResult(id = userdata.latched_filter_id,
                                                               success = False))
        return 'next'

class PursueSampleAborted(smach.State):
    def __init__(self, result_pub):
        smach.State.__init__(self,
                             input_keys = ['latched_filter_id'],
                             outcomes=['next'])
        self.result_pub = result_pub
    def execute(self, userdata):
        self.result_pub.publish(samplereturn_msg.PursuitResult(id = userdata.latched_filter_id,
                                                               success = False))
        return 'next'
    
#transforms a detected point, and also calculates a position min_pursuit_distance from the point
def calculate_pursuit(_tf, pursuit_point, min_pursuit_distance, odometry_frame):
    try:
        current_pose = util.get_current_robot_pose(_tf, odometry_frame)
        _tf.waitForTransform(odometry_frame,
                            pursuit_point.header.frame_id,
                            pursuit_point.header.stamp,
                            rospy.Duration(1.0))
        point_in_frame = _tf.transformPoint(odometry_frame, pursuit_point)
        pointing_quat = util.pointing_quaternion_2d(current_pose.pose.position, point_in_frame.point)
        distance_to_point = util.point_distance_2d(current_pose.pose.position, point_in_frame.point)
        pursuit_pose = util.pose_translate_by_quat(current_pose,
                                                   (distance_to_point - min_pursuit_distance),
                                                   pointing_quat)
        #recalculate the quaternion pointing from goal point to sample point
        pointing_quat = util.pointing_quaternion_2d(pursuit_pose.pose.position, point_in_frame.point)
        pursuit_pose.pose.orientation = pointing_quat
        pursuit_pose.header.stamp = rospy.Time(0)
        #rospy.loginfo("CALCULATE PURSUIT: pursuit_point: {!s}, point_in_frame: {!s}, pursuit_pose: {!s}".format(pursuit_point, point_in_frame, pursuit_pose))
        
        return point_in_frame, pursuit_pose
    except tf.Exception, e:
           rospy.logwarn("PURSUE_SAMPLE failed to transform pursuit point %s->%s: %s",
           pursuit_point.header.frame_id, odometry_frame, e)        
