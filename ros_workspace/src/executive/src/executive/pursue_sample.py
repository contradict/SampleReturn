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
import manipulator_msgs.msg as manipulator_msg
import move_base_msgs.msg as move_base_msg
import geometry_msgs.msg as geometry_msg
import samplereturn_msgs.srv as samplereturn_srv

import samplereturn.util as util

from executive.executive_states import DriveToPoseState
from executive.executive_states import PursueDetectedPoint

class PursueSample(object):
    
    def __init__(self):
        
        self.announcer = util.AnnouncerInterface("audio_navigate")
        self.tf_listener = tf.TransformListener()
        self.node_params = util.get_node_params()
        self.move_base = actionlib.SimpleActionClient("planner_move_base",
                                                       move_base_msg.MoveBaseAction)
        
        self.state_machine = smach.StateMachine(
                outcomes=['complete', 'preempted', 'aborted'],
                input_keys = ['action_goal'],
                output_keys = ['action_result'])
        
        self.state_machine.userdata.detected_sample = None
        self.state_machine.userdata.target_sample = None
        self.state_machine.userdata.target_point = None
        self.state_machine.userdata.min_pursuit_distance = 5
        self.state_machine.userdata.max_pursuit_error = 0.2
        self.state_machine.userdata.max_point_lost_time = 20
        self.state_machine.userdata.square_search_size = 0.3
    
        with self.state_machine:
            
                smach.StateMachine.add('START_SAMPLE_PURSUIT',
                                       StartSamplePursuit(self.announcer),
                                       transitions = {'next':'APPROACH_SAMPLE'})
                
                smach.StateMachine.add('APPROACH_SAMPLE',
                                       PursueDetectedPoint(self.announcer,
                                                           self.move_base,
                                                           self.tf_listener,
                                                           within_min_msg = 'Switching to manipulator detection'),
                                       transitions = {'min_distance':'ENABLE_MANIPULATOR_DETECTOR',
                                                      'point_lost':'PURSUE_SAMPLE_ABORTED'},
                                       remapping = {'target_point':'target_sample'})
                
                smach.StateMachine.add('ENABLE_MANIPULATOR_DETECTOR',
                                        smach_ros.ServiceState('enable_manipulator_detector',
                                                                samplereturn_srv.Enable,
                                                                request = samplereturn_srv.EnableRequest(True)),
                                         transitions = {'succeeded':'MANIPULATOR_SEARCH',
                                                        'aborted':'PURSUE_SAMPLE_ABORTED'})
                
                smach.StateMachine.add('MANIPULATOR_SEARCH',
                                       DriveToPoseState(self.move_base, self.tf_listener),
                                       transitions = {'complete':'LOAD_SEARCH_PATH',
                                                      'timeout':'LOAD_SEARCH_PATH',
                                                      'sample_detected':'VISUAL_SERVO'})
                
                smach.StateMachine.add('LOAD_SEARCH_PATH',
                                       LoadSearchPath(self.tf_listener, self.announcer),
                                       transitions = {'next':'DRIVE_SEARCH_PATH',
                                                      'aborted':'PURSUE_SAMPLE_ABORTED',
                                                      'preempted':'PURSUE_SAMPLE_PREEMPTED'})

                smach.StateMachine.add('DRIVE_SEARCH_PATH',
                                       DriveSearchPath(self.announcer),
                                       transitions = {'next_point':'DRIVE_TO_SEARCH_POSE',
                                                      'complete':'complete',
                                                      'preempted':'PURSUE_SAMPLE_PREEMPTED'})
                
                smach.StateMachine.add('DRIVE_TO_SEARCH_POSE',
                                       DriveToPoseState(self.move_base, self.tf_listener),
                                       transitions = {'complete':'DRIVE_SEARCH_PATH',
                                                      'timeout':'DRIVE_SEARCH_PATH',
                                                      'sample_detected':'VISUAL_SERVO'})

                smach.StateMachine.add('VISUAL_SERVO',
                                       VisualServo(self.announcer),
                                       transitions = {'complete':'GRAB_SAMPLE',
                                                      'sample_lost':'LOAD_SEARCH_PATH',
                                                      'preempted':'PURSUE_SAMPLE_PREEMPTED',
                                                      'aborted':'PURSUE_SAMPLE_ABORTED'})  
 
                @smach.cb_interface(input_keys=['detected_sample'])
                def grab_goal_cb(userdata, request):
                    goal = manipulator_msg.ManipulatorGoal()
                    goal.type = goal.GRAB
                    goal.wrist_angle = userdata.detected_sample.grip_angle                    
                    goal.grip_torque = 0.7
                    goal.target_bin = 1
                    return goal
        
                smach.StateMachine.add('GRAB_SAMPLE',
                                       smach_ros.SimpleActionState('manipulator_action',
                                       manipulator_msg.ManipulatorAction,
                                       goal_cb = grab_goal_cb),
                                       transitions = {'succeeded':'CONFIRM_SAMPLE_ACQUIRED',
                                                      'preempted':'PURSUE_SAMPLE_PREEMPTED',
                                                      'aborted':'PURSUE_SAMPLE_ABORTED'})
        
                smach.StateMachine.add('CONFIRM_SAMPLE_ACQUIRED',
                                       ConfirmSampleAcquired(self.announcer),
                                       transitions = {'sample_gone':'DISABLE_MANIPULATOR_DETECTOR',
                                                      'sample_present':'VISUAL_SERVO',
                                                      'preempted':'PURSUE_SAMPLE_PREEMPTED',
                                                      'aborted':'PURSUE_SAMPLE_ABORTED'})
                
                smach.StateMachine.add('DISABLE_MANIPULATOR_DETECTOR',
                                        smach_ros.ServiceState('enable_manipulator_detector',
                                                                samplereturn_srv.Enable,
                                                                request = samplereturn_srv.EnableRequest(False)),
                                         transitions = {'succeeded':'complete',
                                                        'aborted':'PURSUE_SAMPLE_ABORTED'})
        
                smach.StateMachine.add('PURSUE_SAMPLE_PREEMPTED',
                                       PursueSamplePreempted(),
                                       transitions = {'complete':'preempted',
                                                      'fail':'aborted'})
                
                smach.StateMachine.add('PURSUE_SAMPLE_ABORTED',
                                       PursueSampleAborted(),
                                       transitions = {'recover':'START_SAMPLE_PURSUIT',
                                                      'fail':'aborted'})

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
        self.sample_sub_search = rospy.Subscriber('detected_sample_search',
                                                  samplereturn_msg.NamedPoint,
                                                  self.sample_detection_search)
        
        self.sample_sub_manipulator = rospy.Subscriber('detected_sample_manipulator',
                                                        samplereturn_msg.NamedPoint,
                                                        self.sample_detection_manipulator)

        #start action servers and services
        sls.start()
        pursue_sample_server.run_server()
        rospy.spin()
        sls.stop()

    def sample_detection_search(self, sample):
        if sample.name == 'none':
            self.state_machine.userdata.target_sample = None
        else:
            self.state_machine.userdata.target_sample = sample

    def sample_detection_manipulator(self, sample):
        if sample.name == 'none':
            self.state_machine.userdata.detected_sample = None
        else:
            self.state_machine.userdata.detected_sample = sample
    
#searches the globe   
class StartSamplePursuit(smach.State):
    def __init__(self, announcer):
        smach.State.__init__(self,
                             outcomes=['next'],
                             input_keys=['target_sample','action_goal'],
                             output_keys=['target_sample',
                                          'velocity',
                                          'pursue_samples',
                                          'stop_on_sample',
                                          'action_result'])
    
        self.announcer = announcer
    
    def execute(self, userdata):
        
        result = samplereturn_msg.GeneralExecutiveResult('initialized')
        userdata.action_result = result
        userdata.target_sample = None
        userdata.pursue_samples = True
        userdata.stop_on_sample = True
        userdata.velocity = 0.5
        
        while not rospy.is_shutdown():
            if userdata.target_sample is not None:
                break
            rospy.sleep(0.1)
            
        self.announcer.say("Sample detected, pursue ing")               
        return 'next'

class LoadSearchPath(smach.State):
    def __init__(self, listener, announcer):
        smach.State.__init__(self,
                             outcomes=['next', 'preempted', 'aborted'],
                             input_keys=['target_sample'],
                             output_keys=['pose_list'])
    
        self.listener = listener
        self.announcer = announcer

    def execute(self, userdata):

        pose_list = []
        square_step = 2
        start_pose = util.get_current_robot_pose(self.listener)
        rospy.loginfo("SQUARE_SEARCH START POSE: " + str(start_pose))
        next_pose = util.pose_translate(self.listener, start_pose, square_step, 0 )
        next_pose = util.pose_rotate(next_pose, math.pi/2)
        pose_list.append(next_pose)
        next_pose = util.pose_translate(self.listener, start_pose, square_step, square_step )
        next_pose = util.pose_rotate(next_pose, math.pi)        
        pose_list.append(next_pose)
        next_pose = util.pose_translate(self.listener, start_pose, -1*square_step, square_step )
        next_pose = util.pose_rotate(next_pose, math.pi*3/2)                
        pose_list.append(next_pose)
        next_pose = util.pose_translate(self.listener, start_pose, -1*square_step, -1*square_step )
        pose_list.append(next_pose)
        next_pose = util.pose_translate(self.listener, start_pose, square_step, -1*square_step)
        next_pose = util.pose_rotate(next_pose, math.pi/2)   
        pose_list.append(next_pose)
        userdata.pose_list = pose_list
                
        self.announcer.say("No sample found. Searching area")
        
        return 'next'

class DriveSearchPath(smach.State):
    def __init__(self, announcer):
        smach.State.__init__(self,
                             input_keys=['pose_list'],
                             output_keys=['target_pose',
                                          'velocity',
                                          'pursue_samples',
                                          'pose_list'],
                             outcomes=['next_point',
                                       'complete',
                                       'preempted',
                                       'aborted'])
        
        self.announcer = announcer
            
    def execute(self, userdata):    
        
        if (len(userdata.pose_list) > 0):
            userdata.target_pose = userdata.pose_list.pop(0)
            userdata.velocity = 0.5
            userdata.pursue_samples = True
            return 'next_point'
        else:
            self.announcer.say("No sample found, abort ing pursuit")
            return 'complete'

class VisualServo(smach.State):
    def __init__(self, announcer):
        smach.State.__init__(self,
                             outcomes=['complete', 'sample_lost', 'preempted', 'aborted'])
    
        self.announcer = announcer

        self.servo = actionlib.SimpleActionClient("visual_servo_action",
                                                   visual_servo_msg.VisualServoAction)
        
    def execute(self, userdata):
        
        self.last_sample_detected = rospy.get_time()
        self.last_servo_feedback = rospy.get_time()
        self.smach_outcome = 'aborted'
        self.servo.wait_for_server(rospy.Duration(5))
        self.servo.send_goal(visual_servo_msg.VisualServoGoal(),
                             feedback_cb=self.servo_feedback_cb)

        self.announcer.say("Sample in manipulator view, servo ing")
    
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            servo_state = self.servo.get_state()
            if servo_state not in util.actionlib_working_states:
                break    
    
        if servo_state == action_msg.GoalStatus.SUCCEEDED:
            self.announcer.say('Servo complete, deploy ing gripper')
            self.smach_outcome = 'complete'
                
        return self.smach_outcome
    
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
             self.smach_outcome = 'sample_lost'

class ConfirmSampleAcquired(smach.State):
    def __init__(self, announcer):
        smach.State.__init__(self,
                             outcomes=['sample_gone',
                                       'sample_present',
                                       'preempted',
                                       'aborted'],
                             input_keys=['detected_sample'])

        self.announcer = announcer
    
    def execute(self, userdata):
        
        self.announcer.say("Sample acquired")
        return 'sample_gone'

#drive to detected sample location        
class GrabSample(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['complete','preempted'])
        
    def execute(self, userdata):
        
        if self.preempt_requested():
            return 'preempted'
    
        return 'sample_acquired'
    
class PursueSamplePreempted(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['complete','fail'])
        
    def execute(self, userdata):
        
        return 'complete'

class PursueSampleAborted(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['recover','fail'])
        
    def execute(self, userdata):
        
        return 'fail'