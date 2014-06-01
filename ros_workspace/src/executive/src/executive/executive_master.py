#!/usr/bin/env python
import sys
import math
import threading

import rospy
import rosnode
import smach
import smach_ros
import actionlib
import tf

import samplereturn.util as util

import actionlib_msgs.msg as action_msg
import std_msgs.msg as std_msg
import sensor_msgs.msg as sensor_msgs
import platform_motion_msgs.msg as platform_msg
import platform_motion_msgs.srv as platform_srv
import manipulator_msgs.msg as manipulator_msg
import geometry_msgs.msg as geometry_msg
import visual_servo_msgs.msg as visual_servo_msg
import dynamic_reconfigure.srv as dynsrv
import dynamic_reconfigure.msg as dynmsg
import samplereturn_msgs.msg as samplereturn_msg

from executive.executive_states import MonitorTopicState
from executive.executive_states import WaitForFlagState

class ExecutiveMaster(object):
    
    def __init__(self):
        
        # ordinary variables
        self.last_servo_feedback = None
        self.lost_sample = None
        self.gpio = None
        self.search_sample = None
        self.manip_sample = None
        
        self.last_detected_mode = None
        self.last_selected_mode = None
        self.selected_mode = None 
        self.preempt_modes = ('LEVEL_ONE_MODE', 'LEVEL_TWO_MODE', 'MANUAL_MODE')
        

        #load all private node parameters
        self.node_params = util.get_node_params()
        self.GPIO_PIN_LEVEL_ONE = int(self.node_params.GPIO_PIN_LEVEL_ONE, 16)
        self.GPIO_PIN_LEVEL_TWO = int(self.node_params.GPIO_PIN_LEVEL_TWO, 16)
        
        # create publishers
        self.announcer = util.AnnouncerInterface("audio_navigate")
  
        self.state_machine = smach.StateMachine(outcomes=['shutdown', 'aborted'],
                                                input_keys = [],
                                                output_keys = [])
        
        #create initial userdata states in this state machine
        self.state_machine.userdata.paused = None
        self.state_machine.userdata.selected_mode = None
        
        #Beginning of top level state machine declaration.
        #In this machine, "preempted" will indicate a normal command
        #from another process or node, requesting the state machine to pause
        #and handle some input.  This will probably be primarily for handling
        #mode changes. "aborted" will indicate an internal failure in a state 
        #that prevents the state from executing properly.
        #The "next" outcome will be used for states that have only one possible transition.
        with self.state_machine: 
            
            smach.StateMachine.add('START_MASTER_EXECUTIVE',
                                    StartState(self.announcer),
                                    transitions = { 'next':'WAIT_FOR_UNPAUSE',
                                                    'preempted':'TOP_PREEMPTED',
                                                    'aborted':'TOP_ABORTED'})

            rospy.loginfo("NODE PARAMS in with self.sm: " + str(self.node_params))
                        
            cam_dict = {'NAV_CENTER' : 'camera_started',
                        'NAV_PORT' : 'camera_started',
                        'NAV_STARBOARD' : 'camera_started'}
            if not self.node_params.ignore_manipulator_camera:
                cam_dict['MANIPULATOR'] = 'camera_started'
            if not self.node_params.ignore_search_camera:
                cam_dict['SEARCH'] = 'camera_started'
            
            #concurrency container to allow waiting for cameras in parallel
            wait_for_cams = smach.Concurrence(outcomes = ['all_started',
                                                          'timeout',
                                                          'preempted'],
                                              default_outcome = 'timeout',
                                              input_keys = [],
                                              output_keys = [],
                                              outcome_map = { 'all_started' : cam_dict})
            
            with wait_for_cams:
                
                camera_wait_outcomes = [('Ready', 'camera_started', 1 )]
                
                smach.Concurrence.add('NAV_CENTER',
                                      MonitorTopicState('navigation_center_camera_status',
                                                        'data',
                                                        std_msg.String,
                                                        camera_wait_outcomes,
                                                        timeout = 10.0))
                                
                smach.Concurrence.add('NAV_PORT',
                                      MonitorTopicState('navigation_port_camera_status',
                                                        'data',
                                                        std_msg.String,
                                                        camera_wait_outcomes,
                                                        timeout = 10.0))
                
                smach.Concurrence.add('NAV_STARBOARD',
                                      MonitorTopicState('navigation_starboard_camera_status',
                                                        'data',
                                                        std_msg.String,
                                                        camera_wait_outcomes,
                                                        timeout = 10.0))
                
                if not self.node_params.ignore_manipulator_camera:
                    smach.Concurrence.add('MANIPULATOR',
                                          MonitorTopicState('manipulator_camera_status',
                                                            'data',
                                                            std_msg.String,
                                                            camera_wait_outcomes,
                                                            timeout = 10.0))
                
                if not self.node_params.ignore_search_camera:    
                    smach.Concurrence.add('SEARCH',
                                          MonitorTopicState('search_camera_status',
                                                            'data',
                                                            std_msg.String,
                                                            camera_wait_outcomes,
                                                            timeout = 10.0))
                
            smach.StateMachine.add('CHECK_CAMERAS',
                                   wait_for_cams,
                                   transitions = {'all_started':'CAMERA_SUCCESS',
                                                  'timeout':'CAMERA_FAILURE',
                                                  'preempted':'TOP_PREEMPTED'})
            
            smach.StateMachine.add('CAMERA_FAILURE',
                                   CameraFailure(output_keys=['camera_status'],
                                                 outcomes=['next']),
                                   transitions = {'next':'CHECK_MODE'})
            
            smach.StateMachine.add('CAMERA_SUCCESS',
                                   CameraSuccess(output_keys=['camera_status'],
                                                 outcomes=['next']),
                                   transitions = {'next':'CHECK_MODE'})
            
            smach.StateMachine.add('WAIT_FOR_UNPAUSE',
                                   WaitForFlagState('paused',
                                                    flag_trigger_value = False,
                                                    timeout = 10,
                                                    announcer = self.announcer,
                                                    start_message ='Waiting for system enable'),
                                   transitions = {'next':'HOME_PLATFORM',
                                                  'timeout':'WAIT_FOR_UNPAUSE',
                                                  'preempted':'TOP_PREEMPTED'})
            
            smach.StateMachine.add('HOME_PLATFORM',
                                   HomePlatform(self.announcer),
                                   transitions = {'homed':'CHECK_MODE',
                                                  'paused':'WAIT_FOR_UNPAUSE',
                                                  'preempted':'TOP_PREEMPTED',
                                                  'aborted':'TOP_ABORTED'})
         
            smach.StateMachine.add('CHECK_MODE',
                                   CheckMode(self.announcer),
                                   transitions = {'manual':'MANUAL_MODE',
                                                  'level_one':'LEVEL_ONE_MODE',
                                                  'level_two':'LEVEL_TWO_MODE',
                                                  'check_cameras':'CHECK_CAMERAS',
                                                  'preempted':'TOP_PREEMPTED',
                                                  'aborted':'TOP_ABORTED'})                       

            smach.StateMachine.add('LEVEL_ONE_MODE',
                                   smach_ros.SimpleActionState('level_one',
                                   samplereturn_msg.GeneralExecutiveAction,
                                   goal = samplereturn_msg.GeneralExecutiveGoal()),
                                   transitions = {'succeeded':'WAIT_FOR_MODE_CHANGE',
                                                  'preempted':'TOP_PREEMPTED',
                                                  'aborted':'TOP_ABORTED'})

            smach.StateMachine.add('LEVEL_TWO_MODE',
                                   smach_ros.SimpleActionState('level_two',
                                   samplereturn_msg.GeneralExecutiveAction,
                                   goal = samplereturn_msg.GeneralExecutiveGoal()),
                                   transitions = {'succeeded':'WAIT_FOR_MODE_CHANGE',
                                                  'preempted':'TOP_PREEMPTED',
                                                  'aborted':'TOP_ABORTED'})
            
            smach.StateMachine.add('WAIT_FOR_MODE_CHANGE',
                                   WaitForFlagState('selected_mode',
                                                    flag_trigger_value = 'manual',
                                                    timeout = 20,
                                                    announcer = self.announcer,
                                                    start_message ='Mode complete, switch to manual'),
                                   transitions = {'next':'CHECK_MODE',
                                                  'timeout':'WAIT_FOR_MODE_CHANGE',
                                                  'preempted':'TOP_PREEMPTED'})
            
            mcg = platform_msg.ManualControlGoal()
            mcg.mode = mcg.FULL_CONTROL
            smach.StateMachine.add('MANUAL_MODE',
                                   smach_ros.SimpleActionState('manual_control',
                                   platform_msg.ManualControlAction,
                                   goal = mcg),
                                   transitions = {'succeeded':'CHECK_MODE',
                                                  'preempted':'TOP_PREEMPTED',
                                                  'aborted':'TOP_ABORTED'})    

            smach.StateMachine.add('TOP_PREEMPTED',
                                   TopPreempted(),
                                   transitions = {'next':'CHECK_MODE'})
            
            smach.StateMachine.add('TOP_ABORTED',
                                   TopAborted(),
                                   transitions = {'recover':'CHECK_MODE',
                                                  'fail':'aborted'})
            
        sls = smach_ros.IntrospectionServer('top_introspection',
                                            self.state_machine,
                                            '/START_MASTER_EXECUTIVE')
        # subscribe to topics
        rospy.Subscriber("gpio_read", platform_msg.GPIO, self.gpio_update)
        rospy.Subscriber("pause_state", std_msg.Bool, self.pause_state_update)
        
        sls.start()
        
        smach_thread = threading.Thread(target=self.state_machine.execute)
        smach_thread.start()
        
        # Wait for ctrl-c
        rospy.spin()
        sls.stop()
        
        # Request the container to preempt
        self.state_machine.request_preempt()
        
        smach_thread.join()

    def start_state_machine(self):
        self.state_machine.execute()
        rospy.spin()

    #topic handlers

    def gpio_update(self, gpio):
        if gpio.servo_id == self.node_params.gpio_servo_id:
            #store the new gpio value, and pass it to any functions that should react
            self.gpio = gpio
            self.handle_mode_switch(gpio.new_pin_states)
                
    def sample_detection_search_update(self, msg):
        self.search_sample = msg

    def sample_detection_manip_update(self, msg):
        self.manip_sample = msg
        
    def pause_state_update(self, msg):
        self.state_machine.userdata.paused = msg.data
                
    #other methods

    def handle_mode_switch(self, pin_states):
        if pin_states&self.GPIO_PIN_LEVEL_ONE == 0:
            detected_mode = "level_one"
        elif pin_states&self.GPIO_PIN_LEVEL_TWO == 0:
            detected_mode = "level_two"
        else:
            detected_mode = "manual"
       
        #it requires two matches to change mode
        detect_match = detected_mode == self.last_detected_mode
        self.last_detected_mode = detected_mode
        
        if (detected_mode != self.selected_mode) and detect_match:
            rospy.loginfo("MASTER EXECUTIVE new mode selected: " + detected_mode)
            self.selected_mode = detected_mode
            self.state_machine.userdata.selected_mode = detected_mode
            active_state = self.state_machine.get_active_states()[0]
            if active_state in self.preempt_modes:
                #if we are in one of the sub state machines, preempt
                rospy.loginfo("MASTER EXECUTIVE requesting preempt on: " + active_state)
                self.state_machine.request_preempt()
                while not rospy.is_shutdown():
                    rospy.sleep(0.1)
                    #wait here and not listen to the mode switch
                    if not self.state_machine.preempt_requested():
                        break
                
#state classes for the top level machine
class StartState(smach.State):
    def __init__(self, announcer):
        smach.State.__init__(self,
                             input_keys=['paused', 'selected_mode'],
                             output_keys=['camera_status', 'paused', 'selected_mode'],
                             outcomes=['next', 'preempted', 'aborted'])
        
        self.announcer = announcer
                       
    def execute(self, userdata):

        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            if ((rospy.get_time() - start_time) > 20.0):
                rospy.logwarn('Timeout waiting for callbacks')
                return 'aborted'
            rospy.loginfo('START_MASTER_EXECUTIVE: ' + str(self.announcer.audio_ready()) + ', ' +
                                                       str(userdata.selected_mode) + ', ' +
                                                       str(userdata.paused))
            if self.announcer.audio_ready() \
               and userdata.selected_mode is not None \
               and userdata.paused is not None:
                break
      
        userdata.camera_status = 'unknown'
        return 'next'
        
#this state homes all mechanisms on the robot platform
class HomePlatform(smach.State):
    def __init__(self, announcer):
        smach.State.__init__(self,
                            input_keys=['camera_status', 'paused', 'selected_mode'],
                            outcomes=['homed', 'paused', 'aborted','preempted'])   
        
        self.announcer = announcer

        self.platform_motion_input_select = \
                rospy.ServiceProxy("CAN_select_motion_mode",
                platform_srv.SelectMotionMode)

        # homing action clients
        self.home_wheelpods = \
                actionlib.SimpleActionClient("home_wheel_pods",
                platform_msg.HomeAction)
        
        self.home_carousel = \
                actionlib.SimpleActionClient("home_carousel",
                platform_msg.HomeAction)
        
        self.manipulator = \
                actionlib.SimpleActionClient('manipulator_action',
                manipulator_msg.ManipulatorAction)
    
    def execute(self, userdata):
       
        #ensure that homing services are live, wait 10 seconds
        start_time = rospy.get_time()    
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            if self.home_wheelpods.wait_for_server(rospy.Duration(0.001))\
            and self.home_carousel.wait_for_server(rospy.Duration(0.001))\
            and self.manipulator.wait_for_server(rospy.Duration(0.001)):
                break #all services up, exit this loop
            if (rospy.get_time() - start_time) > 10.0:
                self.announcer.say("Home ing services not started")
                rospy.logwarn('Timeout waiting for homing services')
                return 'aborted'
            
        #home platform, manipulator first
        self.announcer.say('Home ing')
        mh_msg = manipulator_msg.ManipulatorGoal()
        mh_msg.type = mh_msg.HOME
        self.manipulator.send_goal(mh_msg)
        while not rospy.is_shutdown(): 
            rospy.sleep(0.1)
            manipulator_state = self.manipulator.get_state()
            if manipulator_state not in util.actionlib_working_states:
                break           
        if manipulator_state != action_msg.GoalStatus.SUCCEEDED:             
            if userdata.paused:
                return 'paused' #action server probably returned because of pause
            else:
                rospy.logwarn('Failure during manipulator homing')
                return 'aborted'
                    
        self.platform_motion_input_select(platform_srv.SelectMotionModeRequest.MODE_HOME)    
        rospy.sleep(0.1)
        self.home_wheelpods.send_goal(platform_msg.HomeGoal(home_count=3))
        self.home_carousel.send_goal(platform_msg.HomeGoal(home_count=1))
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            wheelpods_state = self.home_wheelpods.get_state()
            carousel_state = self.home_carousel.get_state()
            if wheelpods_state not in util.actionlib_working_states \
               and carousel_state not in util.actionlib_working_states:
                break
        if wheelpods_state != action_msg.GoalStatus.SUCCEEDED \
           or carousel_state != action_msg.GoalStatus.SUCCEEDED:
            if userdata.paused:
                return 'paused' #action server probably returned because of pause
            else:
                rospy.logwarn("Failure during homing (wheelpods, carousel) : (%d, %d)", wheelpods_state, carousel_state)
                return 'aborted'
               
        return 'homed'
    
class CheckMode(smach.State):
    def __init__(self, announcer):
        smach.State.__init__(self,
                             input_keys=['selected_mode', 'camera_status'],
                             output_keys=['camera_status'],
                             outcomes=['manual', 'level_one', 'level_two', 'check_cameras', 'preempted', 'aborted'])

        self.announcer = announcer
               
    def execute(self, userdata):
        
        if userdata.selected_mode == 'manual':
            userdata.camera_status = 'unknown'
            return 'manual'
        
        if userdata.camera_status == 'error':
            words = 'Camera failure, manual mode only'
            self.announcer.say(words)
            start_time = rospy.get_time()
            while (rospy.get_time() - start_time) < 10:
                if userdata.selected_mode == 'manual': return 'manual'
                rospy.sleep(0.1)
        
        if userdata.selected_mode in ['level_one', 'level_two']:
            if userdata.camera_status in ['unknown', 'error']:
                self.announcer.say("Check ing camera status")
                return 'check_cameras'
            elif userdata.camera_status == 'good':                
                return userdata.selected_mode
            else:
                rospy.logwarn("Unexpected camera_status in executive")
                return 'aborted'
            
class CameraFailure(smach.State):
    def execute(self, userdata):
        userdata.camera_status = 'error'
        return 'next'

class CameraSuccess(smach.State):
    def execute(self, userdata):
        userdata.camera_status = 'good'
        return 'next'
    
class TopPreempted(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next'])
        
    def execute(self, userdata):
        
        self.service_preempt()
        
        return 'next'

class TopAborted(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['recover','fail'])
        
    def execute(self, userdata):
        
        return 'fail'
 


    
