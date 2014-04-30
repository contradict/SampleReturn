#!/usr/bin/env python
import math
import smach
import smach_ros
import rospy
import collections
import threading
import actionlib

import visual_servo_msgs.msg as visual_servo_msg
import samplereturn_msgs.msg as samplereturn_msg

class PursueSample(object):
    
    def __init__(self):
        
        self.state_machine = smach.StateMachine(
                outcomes=['complete', 'preempted', 'aborted'],
                input_keys = ['action_goal'],
                output_keys = ['action_result'])
    
        with self.state_machine:
            
                smach.StateMachine.add('START_SAMPLE_PURSUIT',
                                       StartSamplePursuit(),
                                       transitions = {'manipulator_detection':'VISUAL_SERVO'})
                
                smach.StateMachine.add('VISUAL_SERVO',
                                       smach_ros.SimpleActionState('visual_servo_action',
                                       samplereturn_msg.VisualServoAction,
                                       goal = samplereturn_msg.VisualServoGoal()),
                                       transitions = {'complete':'MANIPULATOR_GRAB',
                                                      'preempted':'PURSUE_SAMPLE_PREEMPTED',
                                                      'aborted':'PURSUE_SAMPLE_ABORTED'})  
        
                smach.StateMachine.add('GRAB_SAMPLE',
                                       GrabSample(),
                                       transitions = { 'sample_acquired':'complete'})
        
                smach.StateMachine.add('PURSUE_SAMPLE_PREEMPTED',
                                       PursueSamplePreempted(),
                                       transitions = {'complete':'VISUAL_SERVO',
                                                      'fail':'aborted'})
                
                smach.StateMachine.add('PURSUE_SAMPLE_ABORTED',
                                       PursueSampleAborted(),
                                       transitions = {'recover':'START_PURSUIT',
                                                      'fail':'aborted'})

        #action server wrapper    
        pursue_sample_server = smach_ros.ActionServerWrapper(
            'pursue_sample', platform_msg.GeneralExecutiveAction,
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

        #start action servers and services
        sls.start()
        pursue_sample_server.run_server()
    
#searches the globe   
class StartSamplePursuit(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['manipulator_detection']
                            )
    def execute(self, userdata):
        
        return 'sample_detected'

#drive to detected sample location        
class GrabSample(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['complete','preempted']
                            )
        
    def execute(self, userdata):
        
        if self.preempt_requested():
            return 'preempted'
    
        return 'sample_acquired'
    
    
class PursueSamplePreempted(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['complete','fail'])
        
    def execute(self):
        
        return 'complete'

class PursueSampleAborted(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['recover','fail'])
        
    def execute(self):
        
        return 'fail'