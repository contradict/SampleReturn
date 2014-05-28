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

import samplereturn.util as util

from executive.executive_states import DriveToPoseState
from executive.executive_states import WaitForFlagState
from executive.executive_states import AnnounceState
from executive.executive_states import GetPursueDetectedPointState
from executive.executive_states import SelectMotionMode


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
        self.state_machine.userdata.search_velocity = self.node_params.search_velocity
        
        #use these
        self.state_machine.userdata.true = True
        self.state_machine.userdata.false = False

        with self.state_machine:
            
            MODE_PLANNER = platform_srv.SelectMotionModeRequest.MODE_PLANNER_TWIST            
            MODE_SERVO = platform_srv.SelectMotionModeRequest.MODE_SERVO            
            
            smach.StateMachine.add('START_SAMPLE_PURSUIT',
                                   StartSamplePursuit(self.announcer),
                                   transitions = {'next':'APPROACH_SAMPLE'})
            

            self.pursue_detected_point = GetPursueDetectedPointState(self.move_base,
                                                                     self.tf_listener)
            
            smach.StateMachine.add('APPROACH_SAMPLE',
                                   self.pursue_detected_point,
                                   transitions = {'min_distance':'ENABLE_MANIPULATOR_DETECTOR',
                                                  'point_lost':'PURSUE_SAMPLE_ABORTED',
                                                  'complete':'PURSUE_SAMPLE_ABORTED',
                                                  'timeout':'PURSUE_SAMPLE_ABORTED',
                                                  'aborted':'PURSUE_SAMPLE_ABORTED'},
                                   remapping = {'velocity':'pursuit_velocity',                
                                                'max_point_lost_time':'max_sample_lost_time',
                                                'pursue_samples':'false'})                
            
            smach.StateMachine.add('ENABLE_MANIPULATOR_DETECTOR',
                                    smach_ros.ServiceState('enable_manipulator_detector',
                                                            samplereturn_srv.Enable,
                                                            request = samplereturn_srv.EnableRequest(True)),
                                     transitions = {'succeeded':'ANNOUNCE_MANIPULATOR_SEARCH',
                                                    'aborted':'PURSUE_SAMPLE_ABORTED'})
            
            smach.StateMachine.add('ANNOUNCE_MANIPULATOR_SEARCH',
                                   AnnounceState(self.announcer,
                                                 'Switch ing to manipulator detection'),
                                   transitions = {'next':'MANIPULATOR_SEARCH'})                
            
            
            smach.StateMachine.add('MANIPULATOR_SEARCH',
                                   DriveToPoseState(self.move_base, self.tf_listener),
                                   transitions = {'complete':'LOAD_SEARCH_PATH',
                                                  'timeout':'LOAD_SEARCH_PATH',
                                                  'sample_detected':'VISUAL_SERVO'},
                                   remapping = {'pursue_samples':'true'})
            
            smach.StateMachine.add('LOAD_SEARCH_PATH',
                                   LoadSearchPath(self.tf_listener, self.announcer),
                                   transitions = {'next':'DRIVE_SEARCH_PATH',
                                                  'aborted':'PURSUE_SAMPLE_ABORTED'})

            smach.StateMachine.add('DRIVE_SEARCH_PATH',
                                   DriveSearchPath(self.announcer),
                                   transitions = {'next_point':'DRIVE_TO_SEARCH_POSE',
                                                  'complete':'PURSUE_SAMPLE_ABORTED'})
            
            smach.StateMachine.add('DRIVE_TO_SEARCH_POSE',
                                   DriveToPoseState(self.move_base, self.tf_listener),
                                   transitions = {'complete':'DRIVE_SEARCH_PATH',
                                                  'timeout':'DRIVE_SEARCH_PATH',
                                                  'sample_detected':'SELECT_SERVO'},
                                   remapping = {'pursue_samples':'true'})
            
            smach.StateMachine.add('SELECT_SERVO',
                                    SelectMotionMode(self.CAN_interface,
                                                     self.announcer,
                                                     MODE_SERVO),
                                    transitions = {'next':'VISUAL_SERVO',
                                                  'failed':'PURSUE_SAMPLE_ABORTED'})    

            smach.StateMachine.add('VISUAL_SERVO',
                                   VisualServo(self.announcer),
                                   transitions = {'complete':'GRAB_SAMPLE',
                                                  'sample_lost':'LOAD_SEARCH_PATH',
                                                  'preempted':'CHECK_PAUSE_STATE',
                                                  'aborted':'ABORT_SERVO'})  

            @smach.cb_interface(input_keys=['latched_sample'])
            def grab_goal_cb(userdata, request):
                goal = manipulator_msg.ManipulatorGoal()
                goal.type = goal.GRAB
                goal.wrist_angle = userdata.latched_sample.grip_angle                    
                goal.grip_torque = 0.7
                goal.target_bin = 1
                return goal
    
            #if Steve pauses the robot during this action, it returns preempted,
            #return to visual servo after pause, in case robot moved
            smach.StateMachine.add('GRAB_SAMPLE',
                                   smach_ros.SimpleActionState('manipulator_action',
                                   manipulator_msg.ManipulatorAction,
                                   goal_cb = grab_goal_cb),
                                   transitions = {'succeeded':'CONFIRM_SAMPLE_ACQUIRED',
                                                  'preempted':'CHECK_PAUSE_STATE',
                                                  'aborted':'ABORT_SERVO'})
            
            #if the grab action is preempted by shutdown (or other non-pause reason),
            #exit pursue sample.  If paused, wait for unpause
            smach.StateMachine.add('CHECK_PAUSE_STATE',
                                   WaitForFlagState('paused',
                                                    flag_trigger_value = True),
                                   transitions = {'next':'WAIT_FOR_UNPAUSE',
                                                  'timeout':'ABORT_SERVO'})
    
            smach.StateMachine.add('WAIT_FOR_UNPAUSE',
                                   WaitForFlagState('paused',
                                                    flag_trigger_value = False,
                                                    timeout = 10,
                                                    announcer = self.announcer,
                                                    start_message ='Pursuit paused, waiting for un pause'),
                                   transitions = {'next':'VISUAL_SERVO',
                                                  'timeout':'WAIT_FOR_UNPAUSE'})

            smach.StateMachine.add('ABORT_SERVO',
                                    SelectMotionMode(self.CAN_interface,
                                                     self.announcer,
                                                     MODE_PLANNER),
                                    transitions = {'next':'PURSUE_SAMPLE_ABORTED',
                                                   'failed':'PURSUE_SAMPLE_ABORTED'})                
    
            smach.StateMachine.add('CONFIRM_SAMPLE_ACQUIRED',
                                   ConfirmSampleAcquired(self.announcer, self.result_pub),
                                   transitions = {'sample_gone':'DESELECT_SERVO',
                                                  'sample_present':'VISUAL_SERVO',
                                                  'preempted':'PURSUE_SAMPLE_ABORTED',
                                                  'aborted':'PURSUE_SAMPLE_ABORTED'})
            
            smach.StateMachine.add('DISABLE_MANIPULATOR_DETECTOR',
                                    smach_ros.ServiceState('enable_manipulator_detector',
                                                            samplereturn_srv.Enable,
                                                            request = samplereturn_srv.EnableRequest(False)),
                                     transitions = {'succeeded':'DESELECT_SERVO',
                                                    'aborted':'PURSUE_SAMPLE_ABORTED'})

            smach.StateMachine.add('DESELECT_SERVO',
                                    SelectMotionMode(self.CAN_interface,
                                                     self.announcer,
                                                     MODE_PLANNER),
                                    transitions = {'next':'complete',
                                                  'failed':'PURSUE_SAMPLE_ABORTED'})      

            smach.StateMachine.add('PURSUE_SAMPLE_ABORTED',
                                   PursueSampleAborted(self.result_pub),
                                   transitions = {'recover':'START_SAMPLE_PURSUIT',
                                                  'fail':'complete'})


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

    def pause_state_update(self, msg):
        self.state_machine.userdata.paused = msg.data
        self.pursue_detected_point.userdata.paused = msg.data
            
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
                                         'search_velocity',
                                         'action_goal'],
                             output_keys=['velocity',
                                          'pursuit_point',
                                          'stop_on_sample',
                                          'action_result'])
    
        self.announcer = announcer
    
    def execute(self, userdata):
        
        result = samplereturn_msg.GeneralExecutiveResult()
        result.result_string = 'initialized'        
        userdata.action_result = result
        userdata.stop_on_sample = True
        #default velocity for all moves is search velocity,
        #initial approach pursuit done at pursuit_velocity
        userdata.velocity = userdata.search_velocity
        rospy.loginfo("SAMPLE_PURSUIT input_point: %s" % (userdata.action_goal.input_point))
        userdata.pursuit_point = userdata.action_goal.input_point
            
        self.announcer.say("Sample detected, pursue ing")               
        return 'next'

class LoadSearchPath(smach.State):
    def __init__(self, listener, announcer):
        smach.State.__init__(self,
                             outcomes=['next', 'preempted', 'aborted'],
                             input_keys=['square_search_size',
                                         'target_sample'],
                             output_keys=['pose_list'])
    
        self.listener = listener
        self.announcer = announcer

    def execute(self, userdata):

        pose_list = []
        square_step = userdata.square_search_size
        start_pose = util.get_current_robot_pose(self.listener)
        rospy.loginfo("SQUARE_SEARCH START POSE: " + str(start_pose))
        try:
            next_pose = util.translate_base_link(self.listener, start_pose, square_step, 0 )
            next_pose = util.pose_rotate(next_pose, math.pi/2)
            pose_list.append(next_pose)
            next_pose = util.translate_base_link(self.listener, start_pose, square_step, square_step )
            next_pose = util.pose_rotate(next_pose, math.pi)        
            pose_list.append(next_pose)
            next_pose = util.translate_base_link(self.listener, start_pose, -1*square_step, square_step )
            next_pose = util.pose_rotate(next_pose, math.pi*3/2)                
            pose_list.append(next_pose)
            next_pose = util.translate_base_link(self.listener, start_pose, -1*square_step, -1*square_step )
            pose_list.append(next_pose)
            next_pose = util.translate_base_link(self.listener, start_pose, square_step, -1*square_step)
            next_pose = util.pose_rotate(next_pose, math.pi/2)
        except tf.Exception:
            rospy.logwarn("PURSUE_SAMPLE failed to transform robot pose in LoadSearchPath")
            return 'aborted'
        pose_list.append(next_pose)
        userdata.pose_list = pose_list
                
        self.announcer.say("No sample found. Searching area")
        
        return 'next'

class DriveSearchPath(smach.State):
    def __init__(self, announcer):
        smach.State.__init__(self,
                             input_keys=['pose_list'],
                             output_keys=['target_pose',
                                          'pose_list'],
                             outcomes=['next_point',
                                       'complete',
                                       'aborted'])
        
        self.announcer = announcer
            
    def execute(self, userdata):    
        
        if (len(userdata.pose_list) > 0):
            userdata.target_pose = userdata.pose_list.pop(0)
            return 'next_point'
        else:
            self.announcer.say("No sample found, abort ing pursuit")
            return 'complete'

class VisualServo(smach.State):
    def __init__(self, announcer):
        smach.State.__init__(self,
                             output_keys = ['latched_sample'],
                             input_keys = ['detected_sample',
                                           'paused'],
                             outcomes=['complete',
                                       'sample_lost',
                                       'preempted',
                                       'aborted'])
    
        self.announcer = announcer

        self.servo = actionlib.SimpleActionClient("visual_servo_action",
                                                   visual_servo_msg.VisualServoAction)
        self.sample_lost = False
        
    def execute(self, userdata):
        
        self.sample_lost = False
        self.last_sample_detected = rospy.get_time()
        self.last_servo_feedback = rospy.get_time()
        self.servo.wait_for_server(rospy.Duration(5))
        self.servo.send_goal(visual_servo_msg.VisualServoGoal(),
                             feedback_cb=self.servo_feedback_cb)

        self.announcer.say("Sample in manipulator view, servo ing")
    
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            #if we are paused or preempted exit with preempted
            if self.preempt_requested() or userdata.paused:    
                self.servo.cancel_all_goals()
                self.service_preempt()
                return 'preempted' 
            servo_state = self.servo.get_state()
            if servo_state not in util.actionlib_working_states:
                break
            
        if self.sample_lost:
            return 'sample_lost'
    
        if servo_state == action_msg.GoalStatus.SUCCEEDED:
            self.announcer.say('Servo complete, deploy ing gripper')
            userdata.latched_sample = userdata.detected_sample
            return 'complete'
        else:
            self.announcer.say('Visual servo unexpected failure')
            return 'aborted'
                
        return 'aborted'
    
    def servo_feedback_cb(self, feedback):
         this_time = rospy.get_time()
         if feedback.state != feedback.STOP_AND_WAIT:
             self.last_sample_detected = this_time
         delta_time = (this_time - self.last_servo_feedback)
         if delta_time > 4.0:
             if feedback.state == feedback.STOP_AND_WAIT:
                 self.announcer.say("Sample lost")
             else:
                 self.announcer.say("range %d"%(10*int(feedback.error/10)))
             self.last_servo_feedback = this_time
         if (this_time - self.last_sample_detected) > 15.0:
             self.servo.cancel_all_goals()
             self.announcer.say('Canceling visual servo')
             self.sample_lost = True

class ConfirmSampleAcquired(smach.State):
    def __init__(self, announcer, result_pub):
        smach.State.__init__(self,
                             outcomes=['sample_gone',
                                       'sample_present',
                                       'preempted',
                                       'aborted'],
                             input_keys=['latched_sample',
                                        'detected_sample'],
                             output_keys=['detected_sample'])

        self.announcer = announcer
        self.result_pub = result_pub
    
    def execute(self, userdata):
        
        #wait for 1 second, see if sample is present in view
        userdata.detected_sample = None
        rospy.sleep(1.0)
        if userdata.detected_sample is None:
            self.announcer.say("Sample acquired")
            self.result_pub.publish(samplereturn_msg.PursuitResult(True))
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
        smach.State.__init__(self, outcomes=['recover','fail'])

        self.result_pub = result_pub
        
    def execute(self, userdata):

        self.result_pub.publish(samplereturn_msg.PursuitResult(False))
        
        return 'fail'