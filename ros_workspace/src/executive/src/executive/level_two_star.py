#!/usr/bin/env python
import math
import smach
import smach_ros
import rospy
import collections
import threading
import actionlib
import tf

import std_msgs.msg as std_msg
import visual_servo_msgs.msg as visual_servo_msg
import samplereturn_msgs.msg as samplereturn_msg
import move_base_msgs.msg as move_base_msg
import geometry_msgs.msg as geometry_msg

from executive.executive_states import DriveToPoseState
from executive.executive_states import PursueDetectedPoint

import samplereturn.util as util

class LevelTwoStar(object):
    
    def __init__(self):
        
        self.state_machine = smach.StateMachine(
                outcomes=['complete', 'preempted', 'aborted'],
                input_keys = ['action_goal'],
                output_keys = ['action_result'])
    
        with self.state_machine:
            
                smach.StateMachine.add('START_LEVEL_TWO',
                                       StartLeveLTwo(),
                                       transitions = {'next':'GLOBAL_SEARCH'})
                
                smach.StateMachine.add('GLOBAL_SEARCH',
                                       GlobalSearch(),
                                       transitions = {'sample_acquired':'RETURN_TO_SEARCH',
                                                      'preempted':'LEVEL_TWO_PREEMPTED'})
        
                smach.StateMachine.add('RETURN_TO_SEARCH',
                                       ReturnToSearch(),
                                       transitions = { 'complete':'GLOBAL_SEARCH'})
        
                smach.StateMachine.add('LEVEL_TWO_PREEMPTED',
                                      LevelTwoPreempted(),
                                       transitions = {'complete':'preempted',
                                                      'fail':'aborted'})
                
                smach.StateMachine.add('LEVEL_TWO_ABORTED',
                                       LevelTwoAborted(),
                                       transitions = {'recover':'GLOBAL_SEARCH',
                                                      'fail':'aborted'})
                
        #action server wrapper    
        level_two_server = smach_ros.ActionServerWrapper(
            'level_two', platform_msg.GeneralExecutiveAction,
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

        #start action servers and services
        sls.start()
        level_two_server.run_server()
    
#searches the globe   
class StartLeveLTwo(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['sample_detected']
                            )
    def execute(self, userdata):
        
        return 'sample_detected'

#drive to detected sample location        
class GlobalSearch(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['sample_acquired','preempted']
                            )
        
    def execute(self, userdata):
        
        if self.preempt_requested():
            return 'preempted'
    
        return 'sample_acquired'
  
    
class ReturnToSearch(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['complete']
                            )
    def execute(self, userdata):
        
        return 'complete'
    
class LevelTwoPreempted(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['complete','fail'])
        
    def execute(self):
        
        return 'complete'

class LevelTwoAborted(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['recover','fail'])
        
    def execute(self):
        
        return 'fail'