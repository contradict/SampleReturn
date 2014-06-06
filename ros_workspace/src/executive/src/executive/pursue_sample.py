#!/usr/bin/env python
import math
import collections
import threading
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
import samplereturn.simple_motion as simple_motion
from samplereturn.simple_motion import TimeoutException

from executive.executive_states import DriveToPoseState
from executive.executive_states import WaitForFlagState
from executive.executive_states import AnnounceState
from executive.executive_states import GetPursueDetectedPointState
from executive.executive_states import SelectMotionMode
from executive.executive_states import GetSimpleMoveState

class PursueSample(object):
    
    def __init__(self):
        
        rospy.on_shutdown(self.shutdown_cb)
        self.node_params = util.get_node_params()
        self.tf_listener = tf.TransformListener()
        self.executive_frame = self.node_params.executive_frame
        
        #interfaces
        self.announcer = util.AnnouncerInterface("audio_navigate")
        self.result_pub = rospy.Publisher('pursuit_result', samplereturn_msg.PursuitResult)
        self.light_pub = rospy.Publisher('search_lights', std_msg.Bool)
        self.CAN_interface = util.CANInterface()
        
        #get a simple_mover, it's parameters are inside a rosparam tag for this node
        self.simple_mover = simple_motion.SimpleMover('~sample_search_params/',
                                                      self.tf_listener,
                                                      self.check_for_stop)
        
        #strafe definitions, offset is length along strafe line
        #the yaws have a static angle, which the direction from base_link the robot strafes
        self.strafes = rospy.get_param('strafes')
        self.strafes['center']['distance'] = self.node_params.pursuit_step
        
        #for this state machine, there is no preempt path.  It either finshes successfully and
        #reports success on the PursuitResult topic, or in case of any interupption or failure, it
        #exits through the abort path, reporting failure on the topic.
        self.state_machine = smach.StateMachine(
                outcomes=['complete', 'preempted', 'aborted'],
                input_keys = ['action_goal'],
                output_keys = ['action_result'])
        
        self.state_machine.userdata.executive_frame = self.executive_frame
        self.state_machine.userdata.strafes = self.strafes
        self.state_machine.userdata.active_strafe_key = None
        
        self.state_machine.userdata.paused = False        
        self.state_machine.userdata.detected_sample = None
        self.state_machine.userdata.target_sample = None
        self.state_machine.userdata.search_point = None
        self.state_machine.latched_sample = None
        
        self.state_machine.userdata.pursuit_velocity = self.node_params.pursuit_velocity
        self.state_machine.userdata.pursuit_step = self.node_params.pursuit_step

        self.state_machine.userdata.max_pursuit_error = self.node_params.max_pursuit_error       
        self.state_machine.userdata.min_pursuit_distance = self.node_params.min_pursuit_distance
        self.state_machine.userdata.max_sample_lost_time = self.node_params.max_sample_lost_time        

        #strafe search and servo settings, search_count limits the number of times we can enter the
        #sample_lost search pattern
        self.state_machine.userdata.square_search_size = self.node_params.square_search_size
        self.state_machine.userdata.settle_time = 5
        self.state_machine.userdata.manipulator_search_angle = math.pi/4
        self.state_machine.userdata.search_count = 0
        self.state_machine.userdata.search_try_limit = 2
        self.state_machine.userdata.manipulator_correction = self.node_params.manipulator_correction
        self.state_machine.userdata.servo_params = self.node_params.servo_params
        
        #use these as booleans in remaps
        self.state_machine.userdata.true = True
        self.state_machine.userdata.false = False

        #motion mode stuff
        planner_mode = self.node_params.planner_mode
        MODE_PLANNER = getattr(platform_srv.SelectMotionModeRequest, planner_mode)
        MODE_SERVO = platform_srv.SelectMotionModeRequest.MODE_SERVO            

        with self.state_machine:
            
            smach.StateMachine.add('START_SAMPLE_PURSUIT',
                                   StartSamplePursuit(self.announcer),
                                   transitions = {'next':'APPROACH_SAMPLE'})

            smach.StateMachine.add('APPROACH_SAMPLE',
                                   ApproachPoint(self.tf_listener),
                                   transitions = {'complete':'ANNOUNCE_MANIPULATOR_APPROACH',
                                                  'move':'APPROACH_MOVE',
                                                  'return_to_start':'PURSUE_SAMPLE_ABORTED',
                                                  'timeout':'PURSUE_SAMPLE_ABORTED',
                                                  'aborted':'PURSUE_SAMPLE_ABORTED'})
                                               
            smach.StateMachine.add('APPROACH_MOVE',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'APPROACH_SAMPLE',
                                                  'timeout':'APPROACH_SAMPLE',
                                                  'aborted':'APPROACH_SAMPLE'})
           
            smach.StateMachine.add('ANNOUNCE_MANIPULATOR_APPROACH',
                                   AnnounceState(self.announcer,
                                                 'Calculate ing manipulator approach'),
                                   transitions = {'next':'ENABLE_MANIPULATOR_DETECTOR'})                

            smach.StateMachine.add('ENABLE_MANIPULATOR_DETECTOR',
                                    smach_ros.ServiceState('enable_manipulator_detector',
                                                            samplereturn_srv.Enable,
                                                            request = samplereturn_srv.EnableRequest(True)),
                                     transitions = {'succeeded':'SELECT_SERVO',
                                                    'aborted':'PURSUE_SAMPLE_ABORTED'})
            
            smach.StateMachine.add('SELECT_SERVO',
                                   SelectMotionMode(self.CAN_interface,
                                                    MODE_SERVO),
                                   transitions = {'next':'GET_SAMPLE_STRAFE_MOVE',
                                                  'failed':'PURSUE_SAMPLE_ABORTED'})

            #calculate the strafe move to the sample
            smach.StateMachine.add('GET_SAMPLE_STRAFE_MOVE',
                                   GetSampleStrafeMove(self.tf_listener),
                                   transitions = {'strafe':'MANIPULATOR_APPROACH',
                                                  'point_lost':'ANNOUNCE_ABORT'})
            
            smach.StateMachine.add('MANIPULATOR_APPROACH',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'GET_SEARCH_POINTS',
                                                  'timeout':'PURSUE_SAMPLE_ABORTED',
                                                  'aborted':'ANNOUNCE_ABORT'})
            
            smach.StateMachine.add('GET_SEARCH_POINTS',
                                   GetSearchPoints(self.tf_listener, self.announcer),
                                   transitions = {'next':'HANDLE_SEARCH_MOVES',
                                                  'aborted':'ANNOUNCE_ABORT'})

            smach.StateMachine.add('HANDLE_SEARCH_MOVES',
                                   HandleSearchMoves(self.tf_listener, self.announcer),
                                   transitions = {'next_point':'STRAFE_TO_SEARCH_POINT',
                                                  'sample_detected':'VISUAL_SERVO',
                                                  'complete':'ANNOUNCE_ABORT'})
           
            smach.StateMachine.add('STRAFE_TO_SEARCH_POINT',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'HANDLE_SEARCH_MOVES',
                                                  'timeout':'HANDLE_SEARCH_MOVES',
                                                  'aborted':'ANNOUNCE_ABORT'})
          
            #calculate the strafe move to the sample
            smach.StateMachine.add('VISUAL_SERVO',
                                   ServoController(self.tf_listener, self.announcer),
                                   transitions = {'move':'SERVO_MOVE',
                                                  'complete':'GRAB_SAMPLE',
                                                  'point_lost':'GET_SEARCH_POINTS',
                                                  'aborted':'ANNOUNCE_ABORT'})

            smach.StateMachine.add('SERVO_MOVE',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'VISUAL_SERVO',
                                                  'timeout':'VISUAL_SERVO',
                                                  'aborted':'ANNOUNCE_ABORT'})

            @smach.cb_interface(input_keys=['latched_sample'])
            def grab_goal_cb(userdata, request):
                goal = manipulator_msg.ManipulatorGoal()
                goal.type = goal.GRAB
                goal.wrist_angle = userdata.latched_sample.grip_angle                    
                goal.target_bin = userdata.latched_sample.sample_id
                goal.grip_torque = 0.7
                return goal
    
            #if Steve pauses the robot during this action, it returns preempted,
            #return to visual servo after pause, in case robot moved
            smach.StateMachine.add('GRAB_SAMPLE',
                                   smach_ros.SimpleActionState('manipulator_action',
                                   manipulator_msg.ManipulatorAction,
                                   goal_cb = grab_goal_cb),
                                   transitions = {'succeeded':'CONFIRM_SAMPLE_ACQUIRED',
                                                  'preempted':'CHECK_PAUSE_STATE',
                                                  'aborted':'ANNOUNCE_ABORT'})

            smach.StateMachine.add('CONFIRM_SAMPLE_ACQUIRED',
                                   ConfirmSampleAcquired(self.announcer, self.result_pub),
                                   transitions = {'sample_gone':'DISABLE_MANIPULATOR_DETECTOR',
                                                  'sample_present':'VISUAL_SERVO',
                                                  'aborted':'PURSUE_SAMPLE_ABORTED'})

            #if the grab action is preempted by shutdown (or other non-pause reason),
            #exit pursue sample.  If paused, wait for unpause
            smach.StateMachine.add('CHECK_PAUSE_STATE',
                                   WaitForFlagState('paused',
                                                    flag_trigger_value = True),
                                   transitions = {'next':'WAIT_FOR_UNPAUSE',
                                                  'timeout':'ANNOUNCE_ABORT',
                                                  'preempted':'ANNOUNCE_ABORT'})
            
            smach.StateMachine.add('WAIT_FOR_UNPAUSE',
                                   WaitForFlagState('paused',
                                                    flag_trigger_value = False,
                                                    timeout = 20,
                                                    announcer = self.announcer,
                                                    start_message ='Pursuit paused, waiting for un pause'),
                                   transitions = {'next':'VISUAL_SERVO',
                                                  'timeout':'WAIT_FOR_UNPAUSE',
                                                  'preempted':'ANNOUNCE_ABORT'})

            #beginning of clean exit path
            smach.StateMachine.add('DISABLE_MANIPULATOR_DETECTOR',
                                    smach_ros.ServiceState('enable_manipulator_detector',
                                                            samplereturn_srv.Enable,
                                                            request = samplereturn_srv.EnableRequest(False)),
                                     transitions = {'succeeded':'DESELECT_SERVO',
                                                    'aborted':'DESELECT_SERVO'})

            smach.StateMachine.add('DESELECT_SERVO',
                                    SelectMotionMode(self.CAN_interface,
                                                     MODE_PLANNER),
                                    transitions = {'next':'complete',
                                                  'failed':'complete'})      
            
            #beginning of abort path, failing above should enter here at appropriate point
            #to return system to entry state
            smach.StateMachine.add('ANNOUNCE_ABORT',
                                   AnnounceState(self.announcer,
                                                 'Abort ing pursuit'),
                                   transitions = {'next':'ABORT_SERVO'})               

            smach.StateMachine.add('ABORT_SERVO',
                                    SelectMotionMode(self.CAN_interface,
                                                     MODE_PLANNER),
                                    transitions = {'next':'ABORT_MANIPULATOR_DETECTOR',
                                                   'failed':'ABORT_MANIPULATOR_DETECTOR'})                

            smach.StateMachine.add('ABORT_MANIPULATOR_DETECTOR',
                                    smach_ros.ServiceState('enable_manipulator_detector',
                                                            samplereturn_srv.Enable,
                                                            request = samplereturn_srv.EnableRequest(False)),
                                     transitions = {'succeeded':'PURSUE_SAMPLE_ABORTED',
                                                    'aborted':'PURSUE_SAMPLE_ABORTED'})

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
        
    def check_for_stop(self):
        
        active_strafe_key = self.state_machine.userdata.active_strafe_key
        
        #stop if we are blocked in strafe direction
        if active_strafe_key is not None:
            if self.strafes[active_strafe_key]['blocked']:
                rospy.loginfo("PURSUE SAMPLE stopping simple_mover on strafe %s blocked" %(active_strafe_key))
                return True
            #stop if we are using obstacle avoidance strafing and within min pursuit distance
            robot_point = util.get_current_robot_pose(self.tf_listener).pose.position
            sample_point = self.state_machine.userdata.target_sample.point
            distance_to_sample = util.point_distance_2d(robot_point, sample_point)
            self.state_machine.userdata.distance_to_sample = distance_to_sample
            if (distance_to_sample <= self.state_machine.userdata.min_pursuit_distance):
                rospy.loginfo("PURSUE SAMPLE stopping simple_move on pursuit distance = %.3f" %(distance_to_sample))
                return True
        
        #stop if we are moving and looking for manipulator detections
        if self.state_machine.userdata.stop_on_sample and \
           self.state_machine.userdata.detected_sample is not None:
            rospy.loginfo("PURSUE SAMPLE stopping simple_mover on manipulator detection")
            return True
                    
        return False

    def sample_detection_search(self, sample):
            try:
                self.tf_listener.waitForTransform(self.executive_frame,
                                                  sample.header.frame_id,
                                                  sample.header.stamp,
                                                  rospy.Duration(1.0))
                point_in_frame = self.tf_listener.transformPoint(self.executive_frame, sample)
                sample.header = point_in_frame.header
                sample.point = point_in_frame.point
                self.state_machine.userdata.target_sample = sample
            except tf.Exception:
                rospy.logwarn("PURSUE_SAMPLE failed to transform search detection point!")

    def sample_detection_manipulator(self, sample):
        self.state_machine.userdata.detected_sample = sample
 
    def pause_state_update(self, msg):
        self.state_machine.userdata.paused = msg.data
            
    def shutdown_cb(self):
        self.state_machine.request_preempt()
        while self.state_machine.is_running():
            rospy.sleep(0.1)
    
#searches the globe   
class StartSamplePursuit(smach.State):
    def __init__(self, announcer):
        smach.State.__init__(self,
                             outcomes=['next'],
                             input_keys=['target_sample',
                                         'pursuit_velocity',
                                         'action_goal',
                                         'pursuit_point'],
                             output_keys=['velocity',
                                          'approach_points',
                                          'pursuit_point',
                                          'last_sample_detection_time',
                                          'stop_on_sample',
                                          'action_result',
                                          'search_count',
                                          'simple_move',
                                          'distance_to_sample'])
  
        self.announcer = announcer
    
    def execute(self, userdata):
        
        self.announcer.say("Sample detected, pursue ing")
        
        result = samplereturn_msg.GeneralExecutiveResult()
        result.result_string = 'approach failed'
        result.result_int = samplereturn_msg.NamedPoint.NONE
        userdata.action_result = result
        #leave this false until we are in manipulator view
        userdata.stop_on_sample = False
        #default velocity for all moves is in simple_mover,
        #initial approach pursuit done at pursuit_velocity
        userdata.search_count = 0
        rospy.loginfo("SAMPLE_PURSUIT input_point: %s" % (userdata.action_goal.input_point))
        userdata.pursuit_point = userdata.action_goal.input_point
        userdata.approach_points = []
        userdata.distance_to_sample = 20 #maximum possible detection
 
        return 'next'
 
class ApproachPoint(smach.State):
    def __init__(self, tf_listener):
        smach.State.__init__(self,
                             outcomes=['complete',
                                       'move',
                                       'timeout',
                                       'return_to_start',
                                       'aborted'],
                             input_keys=['strafes',
                                         'target_sample',
                                         'approach_points',
                                         'offset_count',
                                         'active_strafe_key',
                                         'min_pursuit_distance',
                                         'pursuit_step',
                                         'pursuit_velocity',
                                         'distance_to_sample'],
                             output_keys=['simple_move',
                                          'approach_points',
                                          'offset_count',
                                          'active_strafe_key'])
        
        self.tf_listener = tf_listener
    
    def execute(self, userdata):
        
        #get all the relationships between robot and sample, angle, distance, etc.
        current_pose = util.get_current_robot_pose(self.tf_listener)
        sample_point = userdata.target_sample.point
        robot_point = current_pose.pose.position
        yaw_to_sample = util.pointing_yaw(robot_point, sample_point)
        actual_yaw = util.get_current_robot_yaw(self.tf_listener)
        rotate_yaw = util.unwind(yaw_to_sample - actual_yaw)
 
        #this is the first move, which is always to point right at the sample
        if len(userdata.approach_points) == 0:
            userdata.offset_count = 0
            userdata.approach_points.append(robot_point)
            return self.spin(rotate_yaw, userdata)
        else:
            #last move was a spin, which is only done to point at the sample
            #time to strafe towards sample
            if userdata.active_strafe_key is None:
                return self.strafe('center', userdata)
            #getting here means last move was a strafe, first check if it was an offset move,
            #which would only be done if center blocked, if so, face the sample again, even if
            #the offset move was blocked, perhaps we got a view on the sample
            elif userdata.active_strafe_key != 'center':
                #append points at ends of offset moves
                userdata.approach_points.append(robot_point)
                return self.spin(rotate_yaw, userdata)
            #last move was not a spin, nor offset, must have been center
            #was it blocked?
            elif userdata.strafes['center']['blocked']:
                #regardless, we got her from a center strafe, so append the point
                userdata.approach_points.append(robot_point)
                #only one offset attempt allowed in sample pursuit, time to leave if it's not zero now
                if offset_count != 0:
                    userdata.active_strafe_key = None
                    return 'return_to_start'
                else:
                    #if left is clear strafe over there
                    if not userdata.strafes['left']['blocked']:
                        #append points at beginning of offsets
                        userdata.approach_points.append(robot_point)
                        return strafe('left', userdata)
                    #if not, then right
                    elif not userdata.strafes['right']['blocked']:
                        #append points at beginning of offsets
                        userdata.approach_points.append(robot_point)
                        return strafe('right', userdata)
                    #all blocked, get the hell out
                    else:
                        userdata.active_strafe_key = None
                        return 'return_to_start'                    
            #center is clear, keep going
            else:
                return self.strafe('center', userdata)
            
            return 'aborted'
 
    def strafe(self, key, userdata):
        #check before any strafe move:
        if userdata.distance_to_sample <= userdata.min_pursuit_distance:
            rospy.loginfo("PURSUIT within minimum distance")
            userdata.active_strafe_key = None
            return 'complete'
        strafe = userdata.strafes[key]
        distance = strafe['distance']
        userdata.offset_count += np.sign(strafe['angle'])
        userdata.simple_move = {'type':'strafe',
                                'angle':strafe['angle'],
                                'distance': distance,
                                'velocity': userdata.pursuit_velocity}        
        userdata.active_strafe_key = key
        return 'move'
    
    def spin(self, angle, userdata):
        userdata.simple_move = {'type':'spin',
                                'angle':angle}
        userdata.active_strafe_key = None
        return 'move'


class ExecuteSimpleMove(smach.State):
    def __init__(self, simple_mover):
        
        smach.State.__init__(self,
                             outcomes=['complete',
                                       'timeout',
                                       'aborted'],
                             input_keys=['simple_move',
                                         'detected_sample'])
        
        self.simple_mover = simple_mover
        
    def execute(self, userdata):
                
        move = userdata.simple_move

        try:
            angle = move['angle']
            if move['type'] == 'spin':
                error = self.simple_mover.execute_spin(angle,
                                                       max_velocity = move.get('velocity'))
                rospy.loginfo("EXECUTED SPIN: %.1f, error %.3f" %( np.degrees(angle),
                                                                   np.degrees(error)))
            elif move['type'] == 'strafe':
                distance = move['distance']
                error = self.simple_mover.execute_strafe(angle,
                                                         move['distance'],
                                                         max_velocity = move.get('velocity'))
                rospy.loginfo("EXECUTED STRAFE angle: %.1f, distance: %.1f, error %.3f" %( np.degrees(angle),
                                                                                           distance,
                                                                                           error))
            else:
                rospy.logwarn('SIMPLE MOTION invalid type')
                return 'aborted'
        
            return 'complete'
            
        except(TimeoutException):
                rospy.logwarn("TIMEOUT during simple_motion.")
                return 'timeout'

        return 'aborted'

class GetSampleStrafeMove(smach.State):
    def __init__(self, tf_listener):
        smach.State.__init__(self,
                             outcomes=['strafe', 'point_lost', 'aborted'],
                             input_keys=['target_sample',
                                         'settle_time'],
                             output_keys=['simple_move',
                                          'target_sample',
                                          'detected_sample',
                                          'stop_on_sample'])
        
        self.tf_listener = tf_listener
        
    def execute(self, userdata):
        
        userdata.target_sample = None
        rospy.sleep(userdata.settle_time)
        if userdata.target_sample is None:
            return 'point_lost'
        else:
            try:
                sample_time = userdata.target_sample.header.stamp
                self.tf_listener.waitForTransform('base_link', 'odom', sample_time, rospy.Duration(1.0))
                point_in_base = self.tf_listener.transformPoint('base_link',
                                                             userdata.target_sample).point
                origin = geometry_msg.Point(0,0,0)
                distance = util.point_distance_2d(origin, point_in_base)
                yaw = util.pointing_yaw(origin, point_in_base)
                userdata.detected_sample = None
                #time to look for sample in manipulator view, stop when it is seen
                userdata.stop_on_sample = True
                userdata.simple_move = {'type':'strafe',
                                        'angle':yaw,
                                        'distance':distance}
                return 'strafe'
            except(tf.Exception):
                rospy.logwarn("PURSUE_SAMPLE failed to get base_link -> odom transform in 1.0 seconds")
                return 'aborted'
        
        return 'aborted'

class GetSearchPoints(smach.State):
    def __init__(self, tf_listener, announcer):
        smach.State.__init__(self,
                             outcomes=['next', 'aborted'],
                             input_keys=['square_search_size',
                                         'search_count',
                                         'search_try_limit'],
                             output_keys=['pose_list',
                                          'search_count'])
    
        self.tf_listener = tf_listener
        self.announcer = announcer

    def execute(self, userdata):

        if userdata.search_count >= userdata.search_try_limit:
            self.announcer.say("Search limit reached")

        pose_list = []
        square_step = userdata.square_search_size

        try:
            start_pose = util.get_current_robot_pose(self.tf_listener)
            next_pose = util.translate_base_link(self.tf_listener, start_pose, square_step, 0)
            pose_list.append(next_pose)
            next_pose = util.translate_base_link(self.tf_listener, start_pose, square_step, square_step)
            pose_list.append(next_pose)
            next_pose = util.translate_base_link(self.tf_listener, start_pose, -square_step, square_step)
            pose_list.append(next_pose)
            next_pose = util.translate_base_link(self.tf_listener, start_pose, -square_step, -square_step)
            pose_list.append(next_pose)
            next_pose = util.translate_base_link(self.tf_listener, start_pose, square_step, -square_step)
            pose_list.append(next_pose)
            userdata.pose_list = pose_list
            self.announcer.say("Search ing area")           
            userdata.search_count += 1
            return 'next'
            
        except tf.Exception:
            rospy.logwarn("PURSUE_SAMPLE failed to transform robot pose in GetSearchMoves")
            return 'aborted'

class HandleSearchMoves(smach.State):
    def __init__(self, tf_listener, announcer):
        smach.State.__init__(self,
                             input_keys=['pose_list',
                                         'detected_sample'],
                             output_keys=['simple_move',
                                          'pose_list',
                                          'stop_on_sample'],
                             outcomes=['next_point',
                                       'sample_detected',
                                       'complete',
                                       'aborted'])
        
        self.tf_listener = tf_listener
        self.announcer = announcer
            
    def execute(self, userdata):    
        
        if userdata.detected_sample is not None:
            #now we go to servo, which should not be interupted by sample detection
            userdata.stop_on_sample = False
            return 'sample_detected'
        
        if (len(userdata.pose_list) > 0):
            target_pose = userdata.pose_list.pop(0)
            search_point = geometry_msg.PointStamped(target_pose.header,
                                                     target_pose.pose.position)
            search_point.header.stamp = rospy.Time(0)            
            try:
                point_in_base = self.tf_listener.transformPoint('base_link', search_point).point                
                origin = geometry_msg.Point(0,0,0)
                distance = util.point_distance_2d(origin, point_in_base)
                yaw = util.pointing_yaw(origin, point_in_base)
                userdata.simple_move = {'type':'strafe',
                                        'angle':yaw,
                                        'distance':distance}
            except(tf.Exception):
                rospy.logwarn("PURSUE_SAMPLE failed to get base_link -> odom transform in 1.0 seconds")
                return 'aborted'
            return 'next_point'
        else:
            self.announcer.say("Search complete, no sample found")
            return 'complete'
        
        return 'aborted'

class ServoController(smach.State):
    def __init__(self, tf_listener, announcer):
        smach.State.__init__(self,
                             outcomes=['move', 'complete', 'point_lost', 'aborted'],
                             input_keys=['detected_sample',
                                         'settle_time',
                                         'manipulator_correction',
                                         'servo_params'],
                             output_keys=['simple_move',
                                          'detected_sample',
                                          'latched_sample'])
        
        self.tf_listener = tf_listener
        self.announcer = announcer
        self.try_count = 0
        
    def execute(self, userdata):
        
        userdata.detected_sample = None
        rospy.sleep(userdata.settle_time)
        if self.try_count > userdata.servo_params['try_limit']:
            self.announcer.say("Servo exceeded try limit")
            rospy.loginfo("SERVO STRAFE failed to hit tolerance before try_limit: %s" % (userdata.servo_params['try_limit']))
            return 'aborted'
        
        if userdata.detected_sample is None:
            self.try_count = 0
            self.announcer.say("Sample lost")
            return 'point_lost'
        else:
            try:
                sample_time = userdata.detected_sample.header.stamp
                self.tf_listener.waitForTransform('manipulator_arm', 'odom', sample_time, rospy.Duration(1.0))
                point_in_manipulator = self.tf_listener.transformPoint('manipulator_arm',
                                                             userdata.detected_sample).point
                point_in_manipulator.x -= userdata.manipulator_correction['x']
                point_in_manipulator.y -= userdata.manipulator_correction['y']
                origin = geometry_msg.Point(0,0,0)
                distance = util.point_distance_2d(origin, point_in_manipulator)
                if distance < userdata.servo_params['final_tolerance']:
                    self.try_count = 0
                    userdata.latched_sample = userdata.detected_sample
                    self.announcer.say("Servo complete at %.1f millimeters. Deploying gripper" % (distance*1000))    
                    return 'complete'
                elif distance < userdata.servo_params['initial_tolerance']:
                    velocity = userdata.servo_params['final_velocity']
                else:
                    velocity = userdata.servo_params['initial_velocity']
                yaw = util.pointing_yaw(origin, point_in_manipulator)
                userdata.simple_move = {'type':'strafe',
                                        'angle':yaw,
                                        'distance':distance,
                                        'velocity':velocity}
                self.announcer.say("Servo ing, distance, %.1f centimeters" % (distance*100))
                rospy.loginfo("DETECTED SAMPLE IN manipulator_arm frame (corrected): " + str(point_in_manipulator))
                self.try_count += 1
                return 'move'
            except tf.Exception:
                rospy.logwarn("MANUAL_CONTROL failed to get manipulator_arm -> odom transform in 1.0 seconds")
                self.try_count = 0
                return 'aborted'
        
        self.try_count = 0
        return 'aborted'

class ConfirmSampleAcquired(smach.State):
    def __init__(self, announcer, result_pub):
        smach.State.__init__(self,
                             outcomes=['sample_gone',
                                       'sample_present',
                                       'aborted'],
                             input_keys=['latched_sample',
                                        'detected_sample',
                                        'action_result'],
                             output_keys=['detected_sample',
                                          'action_result'])

        self.announcer = announcer
        self.result_pub = result_pub
    
    def execute(self, userdata):
        
        #wait for 1 second, see if sample is present in view
        userdata.detected_sample = None
        rospy.sleep(1.0)
        if userdata.detected_sample is None:
            #this is the path of great success, notify filter nodes about success
            #and return the acquired sample ID in case the calling executive needs it
            self.announcer.say("Sample acquired")
            self.result_pub.publish(samplereturn_msg.PursuitResult(True))
            userdata.action_result.result_string = ('sample acquired')
            userdata.action_result.result_int = userdata.latched_sample.sample_id
            return 'sample_gone'
        else:
            if userdata.detected_sample.sample_id == userdata.latched_sample.sample_id:
                self.announcer.say("Sample acquisition failed, retry ing")
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

class PursueSampleAborted(smach.State):
    def __init__(self, result_pub):
        smach.State.__init__(self, outcomes=['next'])

        self.result_pub = result_pub
        
    def execute(self, userdata):

        self.result_pub.publish(samplereturn_msg.PursuitResult(False))
        
        return 'next'