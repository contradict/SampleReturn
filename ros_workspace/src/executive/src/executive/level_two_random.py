#!/usr/bin/env python
import math
import collections
import threading
import random

import smach
import smach_ros
import rospy
import actionlib
import tf

import std_msgs.msg as std_msg
import visual_servo_msgs.msg as visual_servo_msg
import samplereturn_msgs.msg as samplereturn_msg
import move_base_msgs.msg as move_base_msg
import geometry_msgs.msg as geometry_msg
import platform_motion_msgs.msg as platform_msg
import platform_motion_msgs.srv as platform_srv

from executive.executive_states import DriveToPoseState
from executive.executive_states import PursueDetectedPoint
from executive.executive_states import SelectMotionMode
from executive.executive_states import AnnounceState

import samplereturn.util as util

class LevelTwoRandom(object):
    
    def __init__(self):

        self.announcer = util.AnnouncerInterface("audio_navigate")
        self.tf_listener = tf.TransformListener()
        
        self.node_params = util.get_node_params()

        self.move_base = actionlib.SimpleActionClient("planner_move_base",
                                                       move_base_msg.MoveBaseAction)
        self.CAN_interface = util.CANInterface()
        
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            if self.move_base.wait_for_server(rospy.Duration(0.1)):
                break #all services up, exit this loop
            if (rospy.get_time() - start_time) > 10.0:
                self.announcer.say("Move base not available")
                rospy.logwarn('Timeout waiting for move base')
            rospy.sleep(0.5)
        
        self.state_machine = smach.StateMachine(
                outcomes=['complete', 'preempted', 'aborted'],
                input_keys = ['action_goal'],
                output_keys = ['action_result'])
    
        #these are important values!  Must get back to platform in 2 hours
        self.state_machine.userdata.start_time = rospy.Time.now()
        self.state_machine.userdata.return_time = rospy.Time.now() + \
                                                  rospy.Duration(self.node_params.return_time_minutes*60)
        
        #sample pursuit params
        self.state_machine.userdata.max_pursuit_error = self.node_params.max_pursuit_error       
        self.state_machine.userdata.min_pursuit_distance = self.node_params.min_pursuit_distance
        self.state_machine.userdata.max_point_lost_time = self.node_params.max_sample_lost_time
        self.state_machine.userdata.beacon_approach_point = self.node_params.beacon_approach_point
        
        #search line parameters
        self.state_machine.userdata.line_plan_step = self.node_params.line_plan_step
        self.state_machine.userdata.line_replan_distance = self.node_params.line_replan_distance
        self.state_machine.userdata.line_velocity = self.node_params.line_velocity
        self.state_machine.userdata.block_check_distance = self.node_params.block_check_distance
        self.state_machine.userdata.blocked_rotation = self.node_params.blocked_rotation
        self.state_machine.userdata.blocked_rotation_step = self.node_params.blocked_rotation_step
        self.state_machine.userdata.motion_check_interval = self.node_params.motion_check_interval
        self.state_machine.userdata.min_motion = self.node_params.min_motion

        #search line variables
        self.state_machine.userdata.next_line_pose = None
        self.state_machine.userdata.last_line_pose = None
        self.state_machine.userdata.line_yaw = None #IN RADIANS!
        
        self.state_machine.userdata.detected_sample = None
        self.state_machine.userdata.beacon_point = None
        
        #use these
        self.state_machine.userdata.true = True
        self.state_machine.userdata.false = False
    
        with self.state_machine:
            
                smach.StateMachine.add('START_LEVEL_TWO',
                                       StartLeveLTwo(input_keys=['line_yaw',
                                                                 'line_plan_step'],
                                                     output_keys=['action_result',
                                                                  'next_line_pose',
                                                                  'line_yaw',
                                                                  'velocity'],
                                                     outcomes=['next']),
                                       transitions = {'next':'ANNOUNCE_LEVEL_TWO'})
                
                smach.StateMachine.add('ANNOUNCE_LEVEL_TWO',
                                       AnnounceState(self.announcer,
                                                     'Enter ing level two mode'),
                                       transitions = {'next':'SELECT_PLANNER'})
                
                MODE_PLANNER = platform_srv.SelectMotionModeRequest.MODE_PLANNER_TWIST
                smach.StateMachine.add('SELECT_PLANNER',
                                        SelectMotionMode(self.CAN_interface,
                                                         self.announcer,
                                                         MODE_PLANNER),
                                        transitions = {'next':'SEARCH_LINE',
                                                      'failed':'LEVEL_TWO_ABORTED'})    
    
                #the concurrence will preempt all other child states if this cb returns True
                def search_line_cb(outcome_map):
                    return True
                
                search_line = smach.Concurrence(outcomes = ['sample_detected',
                                                            'move_complete',
                                                            'line_blocked',
                                                            'return_home',
                                                            'preempted', 'aborted'],
                                default_outcome = 'aborted',
                                input_keys = ['next_line_pose',
                                              'line_yaw',
                                              'line_velocity',
                                              'line_plan_step',
                                              'line_replan_distance',
                                              'block_check_distance',
                                              'return_time',
                                              'detected_sample',
                                              'motion_check_interval',
                                              'min_motion',
                                              'true', 'false'],
                                output_keys = ['next_line_pose',
                                               'last_line_pose'],
                                child_termination_cb = search_line_cb,
                                outcome_map = { 'sample_detected' : {'DRIVE_TO_POSE':'sample_detected'},
                                                'move_complete' : {'DRIVE_TO_POSE':'complete'},
                                                'line_blocked' : {'SEARCH_LINE_MANAGER':'line_blocked'},
                                                'line_blocked' : {'DRIVE_TO_POSE':'timeout'},
                                                'return_home' : {'SEARCH_LINE_MANAGER':'return_home'},
                                                'preempted' : {'DRIVE_TO_POSE':'preempted',
                                                               'SEARCH_LINE_MANAGER':'preempted'} })
                
                with search_line:
                    
                    smach.Concurrence.add('DRIVE_TO_POSE',
                                           DriveToPoseState(self.move_base, self.tf_listener),
                                           remapping = {'target_pose':'next_line_pose',
                                                        'velocity':'line_velocity',
                                                        'last_pose':'last_line_pose',
                                                        'pursue_samples':'true',
                                                        'stop_on_samples':'false'})

                    smach.Concurrence.add('SEARCH_LINE_MANAGER',
                                          SearchLineManager(self.tf_listener, self.announcer))
                                            
                                            
                smach.StateMachine.add('SEARCH_LINE',
                                       search_line,
                                       transitions = {'sample_detected':'PURSUE_SAMPLE',
                                                      'move_complete':'SEARCH_LINE',
                                                      'line_blocked':'CHOOSE_NEW_LINE',
                                                      'return_home':'START_RETURN_HOME',
                                                      'preempted':'LEVEL_TWO_PREEMPTED',
                                                      'aborted':'LEVEL_TWO_ABORTED'})
                                
                
                @smach.cb_interface(input_keys=['detected_sample'])
                def get_pursuit_goal_cb(userdata, request):
                    goal = samplereturn_msg.GeneralExecutiveGoal()
                    goal.name = userdata.detected_sample.name
                    return goal
                                
                smach.StateMachine.add('PURSUE_SAMPLE',
                                      smach_ros.SimpleActionState('pursue_sample',
                                      samplereturn_msg.GeneralExecutiveAction,
                                      goal_cb = get_pursuit_goal_cb),
                                      transitions = {'succeeded':'RETURN_TO_SEARCH_LINE',
                                                     'preempted':'RETURN_TO_SEARCH_LINE',
                                                     'aborted':'LEVEL_TWO_ABORTED'})
                

                smach.StateMachine.add('RETURN_TO_SEARCH_LINE',
                                       DriveToPoseState(self.move_base,
                                                        self.tf_listener),
                                       transitions = {'complete':'SEARCH_LINE',
                                                      'timeout':'START_RETURN_HOME',
                                                      'sample_detected':'LEVEL_TWO_ABORTED'},
                                       remapping = {'target_pose':'last_line_pose'})
                
                smach.StateMachine.add('CHOOSE_NEW_LINE',
                                       ChooseNewLine(outcomes=['next', 'preempted', 'aborted'],
                                                     input_keys=['line_yaw',
                                                                 'last_line_pose',
                                                                 'line_plan_step',
                                                                 'blocked_rotation'],
                                                     output_keys=['line_yaw',
                                                                  'next_line_pose',
                                                                  'rotate_pose']),
                                       transitions = {'next':'ROTATE_TO_NEW_LINE',
                                                      'preempted':'LEVEL_TWO_PREEMPTED',
                                                      'aborted':'LEVEL_TWO_ABORTED'})
                
                smach.StateMachine.add('ROTATE_TO_NEW_LINE',
                                       DriveToPoseState(self.move_base,
                                                        self.tf_listener),
                                       transitions = {'complete':'SEARCH_LINE',
                                                      'timeout':'CHOOSE_NEW_LINE',
                                                      'sample_detected':'LEVEL_TWO_ABORTED'},
                                       remapping = {'target_pose':'rotate_pose',
                                                    'pursue_samples':'false'})
                
                smach.StateMachine.add('START_RETURN_HOME',
                                       StartReturnHome(self.announcer),
                                       transitions = {'next':'DRIVE_TO_BEACON_APPROACH_START'})
                
                smach.StateMachine.add('DRIVE_TO_BEACON_APPROACH_START',
                                       DriveToPoseState(self.move_base,
                                                        self.tf_listener),
                                       transitions = {'complete':'APPROACH_BEACON',
                                                      'timeout':'START_RETURN_HOME',
                                                      'sample_detected':'LEVEL_TWO_ABORTED'})
                
                smach.StateMachine.add('APPROACH_BEACON',
                       PursueDetectedPoint(self.announcer,
                                           self.move_base,
                                           self.tf_listener,
                                           within_min_msg = 'Reverse ing on to platform'),
                       transitions = {'min_distance':'MOUNT_PLATFORM',
                                      'point_lost':'START_RETURN_HOME'},
                       remapping = {'target_point':'beacon_point'})

                smach.StateMachine.add('MOUNT_PLATFORM',
                                       DriveToPoseState(self.move_base,
                                                        self.tf_listener),
                                       transitions = {'complete':'DESELECT_PLANNER',
                                                      'timeout':'START_RETURN_HOME',
                                                      'sample_detected':'LEVEL_TWO_ABORTED'})

                MODE_ENABLE = platform_srv.SelectMotionModeRequest.MODE_ENABLE
                smach.StateMachine.add('DESELECT_PLANNER',
                                        SelectMotionMode(self.CAN_interface,
                                                         self.announcer,
                                                         MODE_ENABLE),
                                        transitions = {'next':'complete',
                                                      'failed':'LEVEL_TWO_ABORTED'})   
        
                smach.StateMachine.add('LEVEL_TWO_PREEMPTED',
                                      LevelTwoPreempted(),
                                       transitions = {'complete':'preempted',
                                                      'fail':'aborted'})
                
                smach.StateMachine.add('LEVEL_TWO_ABORTED',
                                       LevelTwoAborted(),
                                       transitions = {'recover':'START_RETURN_HOME',
                                                      'fail':'aborted'})
                
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

        #start action servers and services
        sls.start()
        level_two_server.run_server()
        
        self.sample_listener = rospy.Subscriber('detected_sample_search',
                                        samplereturn_msg.NamedPoint,
                                        self.sample_update)
        
        self.beacon_listener = rospy.Subscriber("beacon_pose",
                                                geometry_msg.PoseStamped,
                                                self.beacon_update)

    def sample_update(self, sample):
        if sample.name == 'none':
            self.state_machine.userdata.detected_sample = None
        else:
            self.state_machine.userdata.detected_sample = sample
            
    def beacon_update(self, beacon_pose):
        beacon_pose.header.stamp = rospy.Time(0)
        map_beacon_pose = self.tf_listener.transformPose('/map', beacon_pose)
        header = map_beacon_pose.header
        point = map_beacon_pose.pose.position
        beacon_point = geometry_msg.PointStamped(header, point)
        self.state_machine.userdata.beacon_pose = beacon_point


    
#searches the globe   
class StartLeveLTwo(smach.State):

    def execute(self, userdata):
        
        result = samplereturn_msg.GeneralExecutiveResult('initialized')
        userdata.action_result = result
        userdata.line_yaw = 0
        userdata.velocity = 0.5
        
        header = std_msg.Header(0, rospy.Time(0), '/map')        
        first_pose = geometry_msg.Pose()
        first_pose.position = geometry_msg.Point(userdata.line_plan_step, 0, 0)
        quat_array = tf.transformations.quaternion_from_euler(0, 0,  userdata.line_yaw)           
        first_pose.orientation = geometry_msg.Quaternion(*quat_array)
        userdata.next_line_pose = geometry_msg.PoseStamped(header, first_pose)        

        return 'next'

#drive to detected sample location        
class SearchLineManager(smach.State):
    def __init__(self, listener, announcer):
        smach.State.__init__(self,
                             input_keys = ['next_line_pose',
                                           'line_plan_step',
                                           'line_yaw',
                                           'line_replan_distance',
                                           'block_check_distance',
                                           'return_time'],
                             output_keys = ['next_line_pose'],
                             outcomes=['sample_detected',
                                       'line_blocked',
                                       'return_home',
                                       'preempted', 'aborted'])
        
        self.listener = listener
        self.announcer = announcer
        
    def execute(self, userdata):
    
        while not rospy.is_shutdown():  
            
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'

            current_pose = util.get_current_robot_pose(self.listener)
            distance = util.pose_distance_2d(current_pose, userdata.next_line_pose)
            if distance < userdata.line_replan_distance:
                self.announcer.say("Line is clear, continue ing")
                
                new_pose = util.pose_translate_by_yaw(userdata.next_line_pose,
                                                      userdata.line_plan_step,
                                                      userdata.line_yaw)
                                
                userdata.next_line_pose = new_pose
        
            if rospy.Time.now() > userdata.return_time:
                self.announcer.say("Search time expired")
                return 'return_home'
            
            if self.line_blocked():
               self.announcer.say("Line is blocked, rotate ing to new line")
               return 'line_blocked'   
        
            rospy.sleep(0.2)

    def line_blocked(self):
        
        return False
    
class ChooseNewLine(smach.State):
    
    def execute(self, userdata):
        current_pose = userdata.last_line_pose
        yaw_changes = [math.radians(userdata.blocked_rotation),
                      math.radians(-1*userdata.blocked_rotation)]
        yaw_change = random.choice(yaw_changes)
        line_yaw = userdata.line_yaw + yaw_change
        rotate_pose = util.pose_rotate(current_pose, yaw_change)
        next_line_pose = util.pose_translate_by_yaw(rotate_pose,
                                                    userdata.line_plan_step,
                                                    line_yaw)

        userdata.line_yaw = line_yaw
        userdata.rotate_pose = rotate_pose
        userdata.next_line_pose = next_line_pose
        
        return 'next'
  
class StartReturnHome(smach.State):
 
    def __init__(self, announcer):

        smach.State.__init__(self,
                             outcomes=['next',
                                       'preempted',
                                       'aborted'],
                             input_keys=['beacon_approach_point'],
                             output_keys=['target_pose',
                                          'velocity',
                                          'pursue_samples'])
        
        self.announcer = announcer

    def execute(self, userdata):
        
        self.announcer.say("Return ing to platform")

        header = std_msg.Header(0, rospy.Time(0), '/map')
        #the beacon is probably not in view, drive to a point probably in front of it
        approach_point = userdata.beacon_approach_point
        point = geometry_msg.Point(approach_point['x'],
                                   approach_point['y'],
                                   math.radians(approach_point['yaw']))
        quat_array = tf.transformations.quaternion_from_euler(0, 0, math.pi)           
        pose = geometry_msg.Pose(point, geometry_msg.Quaternion(*quat_array))
                
        userdata.target_pose = geometry_msg.PoseStamped(header, pose)
        userdata.pursue_samples = False
        
        return 'next'
    
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