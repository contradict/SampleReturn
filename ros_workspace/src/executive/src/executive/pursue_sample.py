#!/usr/bin/env python
import math
import collections
import threading

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
        self.move_base = actionlib.SimpleActionClient("planner_move_base",
                                                       move_base_msg.MoveBaseAction)
        self.result_pub = rospy.Publisher('pursuit_result', samplereturn_msg.PursuitResult)
        self.CAN_interface = util.CANInterface()
        
        #get a simple_mover, it's parameters are inside a rosparam tag for this node
        self.simple_mover = simple_motion.SimpleMover('~pursue_sample_params/', self.tf_listener)
        
        #for this state machine, there is no preempt path.  It either finshes successfully and
        #reports success on the PursuitResult topic, or in case of any interupption or failure, it
        #exits through the abort path, reporting failure on the topic.
        self.state_machine = smach.StateMachine(
                outcomes=['complete', 'preempted', 'aborted'],
                input_keys = ['action_goal'],
                output_keys = ['action_result'])
        
        self.state_machine.userdata.executive_frame = self.executive_frame
        self.state_machine.userdata.paused = False        
        self.state_machine.userdata.detected_sample = None
        self.state_machine.userdata.target_sample = None
        self.state_machine.userdata.target_point = None
        self.state_machine.latched_sample = None
        
        self.state_machine.userdata.square_search_size = self.node_params.square_search_size
        self.state_machine.userdata.max_pursuit_error = self.node_params.max_pursuit_error       
        self.state_machine.userdata.min_pursuit_distance = self.node_params.min_pursuit_distance
        self.state_machine.userdata.max_sample_lost_time = self.node_params.max_sample_lost_time        
        self.state_machine.userdata.motion_check_interval = self.node_params.motion_check_interval
        self.state_machine.userdata.min_motion = self.node_params.min_motion
        self.state_machine.userdata.pursuit_velocity = self.node_params.pursuit_velocity

        #strafe search and servo settings, search_count limits the number of times we can enter the
        #sample_lost search pattern
        self.state_machine.userdata.settle_time = 5
        self.state_machine.userdata.simple_move_tolerance = self.node_params.simple_move_tolerance
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

            self.pursue_detected_point = GetPursueDetectedPointState(self.move_base,
                                                                     self.tf_listener)
            
            smach.StateMachine.add('APPROACH_SAMPLE',
                                   self.pursue_detected_point,
                                   transitions = {'min_distance':'ANNOUNCE_MANIPULATOR_APPROACH',
                                                  'point_lost':'PURSUE_SAMPLE_ABORTED',
                                                  'complete':'PURSUE_SAMPLE_ABORTED',
                                                  'timeout':'PURSUE_SAMPLE_ABORTED',
                                                  'aborted':'PURSUE_SAMPLE_ABORTED'},
                                   remapping = {'velocity':'pursuit_velocity',                
                                                'max_point_lost_time':'max_sample_lost_time',
                                                'pursue_samples':'false'})
           
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
            
            self.manipulator_approach = GetSimpleMoveState(self.simple_mover, self.tf_listener)
            
            smach.StateMachine.add('MANIPULATOR_APPROACH',
                                   self.manipulator_approach,
                                   transitions = {'complete':'GET_SEARCH_POINTS',
                                                  'timeout':'PURSUE_SAMPLE_ABORTED',
                                                  'sample_detected':'VISUAL_SERVO',
                                                  'preempted':'ANNOUNCE_ABORT',
                                                  'aborted':'ANNOUNCE_ABORT'},
                                   remapping = {'pursue_samples':'true'})
            
            smach.StateMachine.add('GET_SEARCH_POINTS',
                                   GetSearchPoints(self.tf_listener, self.announcer),
                                   transitions = {'next':'HANDLE_SEARCH_MOVES',
                                                  'aborted':'ANNOUNCE_ABORT'})

            smach.StateMachine.add('HANDLE_SEARCH_MOVES',
                                   HandleSearchMoves(self.tf_listener, self.announcer),
                                   transitions = {'next_point':'STRAFE_TO_SEARCH_POINT',
                                                  'complete':'ANNOUNCE_ABORT'})
           
            self.manipulator_search = GetSimpleMoveState(self.simple_mover, self.tf_listener) 
            
            smach.StateMachine.add('STRAFE_TO_SEARCH_POINT',
                                   self.manipulator_search,
                                   transitions = {'complete':'HANDLE_SEARCH_MOVES',
                                                  'timeout':'HANDLE_SEARCH_MOVES',
                                                  'sample_detected':'VISUAL_SERVO',
                                                  'preempted':'ANNOUNCE_ABORT',
                                                  'aborted':'ANNOUNCE_ABORT'},
                                   remapping = {'pursue_samples':'true'})

            #calculate the strafe move to the sample
            smach.StateMachine.add('VISUAL_SERVO',
                                   ServoController(self.tf_listener, self.announcer),
                                   transitions = {'move':'SERVO_MOVE',
                                                  'complete':'GRAB_SAMPLE',
                                                  'point_lost':'GET_SEARCH_POINTS',
                                                  'aborted':'ANNOUNCE_ABORT'})

            self.servo_move = GetSimpleMoveState(self.simple_mover, self.tf_listener)
            
            smach.StateMachine.add('SERVO_MOVE',
                                   self.servo_move,
                                   transitions = {'complete':'VISUAL_SERVO',
                                                  'timeout':'VISUAL_SERVO',
                                                  'sample_detected':'VISUAL_SERVO',
                                                  'preempted':'ANNOUNCE_ABORT',
                                                  'aborted':'ANNOUNCE_ABORT'},
                                   remapping = {'pursue_samples':'false'})
            
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
                                                  'preempted':'DISABLE_MANIPULATOR_DETECTOR',
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
                self.pursue_detected_point.userdata.pursuit_point = sample
            except tf.Exception:
                rospy.logwarn("PURSUE_SAMPLE failed to transform search detection point!")

    def sample_detection_manipulator(self, sample):
        self.state_machine.userdata.detected_sample = sample
        self.manipulator_approach.userdata.detected_sample = sample
        self.manipulator_search.userdata.detected_sample = sample

    def pause_state_update(self, msg):
        self.state_machine.userdata.paused = msg.data
        self.pursue_detected_point.userdata.paused = msg.data
        self.manipulator_approach.userdata.paused = msg.data
        self.manipulator_search.userdata.paused = msg.data
            
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
                                         'action_goal'],
                             output_keys=['velocity',
                                          'pursuit_point',
                                          'stop_on_sample',
                                          'action_result',
                                          'search_count'])
    
        self.announcer = announcer
    
    def execute(self, userdata):
        
        result = samplereturn_msg.GeneralExecutiveResult()
        result.result_string = 'approach failed'
        result.result_int = samplereturn_msg.NamedPoint.NONE
        userdata.action_result = result
        userdata.stop_on_sample = True
        #default velocity for all moves is search velocity,
        #initial approach pursuit done at pursuit_velocity
        userdata.velocity = userdata.pursuit_velocity
        userdata.search_count = 0
        rospy.loginfo("SAMPLE_PURSUIT input_point: %s" % (userdata.action_goal.input_point))
        userdata.pursuit_point = userdata.action_goal.input_point
            
        self.announcer.say("Sample detected, pursue ing")               
        return 'next'

class GetSampleStrafeMove(smach.State):
    def __init__(self, listener):
        smach.State.__init__(self,
                             outcomes=['strafe', 'point_lost', 'preempted', 'aborted'],
                             input_keys=['target_sample',
                                         'settle_time'],
                             output_keys=['simple_move',
                                          'target_sample'])
        
        self.listener = listener
        
    def execute(self, userdata):
        
        userdata.target_sample = None
        rospy.sleep(userdata.settle_time)
        if userdata.target_sample is None:
            return 'point_lost'
        else:
            try:
                sample_time = userdata.target_sample.header.stamp
                self.listener.waitForTransform('base_link', 'odom', sample_time, rospy.Duration(1.0))
                point_in_base = self.listener.transformPoint('base_link',
                                                             userdata.target_sample).point
                origin = geometry_msg.Point(0,0,0)
                distance = util.point_distance_2d(origin, point_in_base)
                yaw = util.pointing_yaw(origin, point_in_base)
                userdata.simple_move = {'type':'strafe',
                                        'yaw':yaw,
                                        'distance':distance}
                return 'strafe'
            except(tf.Exception):
                rospy.logwarn("PURSUE_SAMPLE failed to get base_link -> odom transform in 1.0 seconds")
                return 'aborted'
        
        return 'aborted'

class GetSearchPoints(smach.State):
    def __init__(self, listener, announcer):
        smach.State.__init__(self,
                             outcomes=['next', 'preempted', 'aborted'],
                             input_keys=['square_search_size',
                                         'search_count',
                                         'search_try_limit'],
                             output_keys=['pose_list',
                                          'search_count'])
    
        self.listener = listener
        self.announcer = announcer

    def execute(self, userdata):

        if userdata.search_count >= userdata.search_try_limit:
            self.announcer.say("Search limit reached")

        pose_list = []
        square_step = userdata.square_search_size

        try:
            start_pose = util.get_current_robot_pose(self.listener)
            next_pose = util.translate_base_link(self.listener, start_pose, square_step, 0)
            pose_list.append(next_pose)
            next_pose = util.translate_base_link(self.listener, start_pose, square_step, square_step)
            pose_list.append(next_pose)
            next_pose = util.translate_base_link(self.listener, start_pose, -square_step, square_step)
            pose_list.append(next_pose)
            next_pose = util.translate_base_link(self.listener, start_pose, -square_step, -square_step)
            pose_list.append(next_pose)
            next_pose = util.translate_base_link(self.listener, start_pose, square_step, -square_step)
            pose_list.append(next_pose)
            userdata.pose_list = pose_list
            self.announcer.say("Search ing area")           
            userdata.search_count += 1
            return 'next'
            
        except tf.Exception:
            rospy.logwarn("PURSUE_SAMPLE failed to transform robot pose in GetSearchMoves")
            return 'aborted'

class HandleSearchMoves(smach.State):
    def __init__(self, listener, announcer):
        smach.State.__init__(self,
                             input_keys=['pose_list'],
                             output_keys=['simple_move',
                                          'pose_list'],
                             outcomes=['next_point',
                                       'complete',
                                       'aborted'])
        
        self.listener = listener
        self.announcer = announcer
            
    def execute(self, userdata):    
        
        if (len(userdata.pose_list) > 0):
            target_pose = userdata.pose_list.pop(0)
            target_point = geometry_msg.PointStamped(target_pose.header,
                                                     target_pose.pose.position)
            target_point.header.stamp = rospy.Time(0)            
            try:
                point_in_base = self.listener.transformPoint('base_link', target_point).point                
                origin = geometry_msg.Point(0.065,0,0)
                distance = util.point_distance_2d(origin, point_in_base)
                yaw = util.pointing_yaw(origin, point_in_base)
                userdata.simple_move = {'type':'strafe',
                                        'yaw':yaw,
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
    def __init__(self, listener, announcer):
        smach.State.__init__(self,
                             outcomes=['move', 'complete', 'point_lost', 'preempted', 'aborted'],
                             input_keys=['detected_sample',
                                         'settle_time',
                                         'manipulator_correction',
                                         'servo_params'],
                             output_keys=['simple_move',
                                          'detected_sample',
                                          'latched_sample'])
        
        self.listener = listener
        self.announcer = announcer
        self.try_count = 0
        
    def execute(self, userdata):
        
        userdata.detected_sample = None
        rospy.sleep(userdata.settle_time)
        if self.try_count > userdata.servo_params['try_limit']:
            self.announcer.say("Servo exceeded try limit")
            rospy.logwarn("SERVO STRAFE failed to hit tolerance before try_limit: " + userdata.servo_params['try_limit'])
            return 'aborted'
        
        if userdata.detected_sample is None:
            self.try_count = 0
            self.announcer.say("Sample lost")
            return 'point_lost'
        else:
            try:
                sample_time = userdata.detected_sample.header.stamp
                self.listener.waitForTransform('manipulator_arm', 'odom', sample_time, rospy.Duration(1.0))
                point_in_manipulator = self.listener.transformPoint('manipulator_arm',
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
                                        'yaw':yaw,
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
                                       'preempted',
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

class PursueSampleAborted(smach.State):
    def __init__(self, result_pub):
        smach.State.__init__(self, outcomes=['next'])

        self.result_pub = result_pub
        
    def execute(self, userdata):

        self.result_pub.publish(samplereturn_msg.PursuitResult(False))
        
        return 'next'