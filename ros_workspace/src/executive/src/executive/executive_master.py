#!/usr/bin/env python
import sys
import math
import rospy
import rosnode
import smach
import smach_ros
import actionlib
import actionlib_msgs.msg as action_msg
import tf
import threading

sys.path.append('/opt/ros/groovy/stacks/audio_common/sound_play/src/')

from samplereturn_msgs.msg import VoiceAnnouncement

import std_msgs.msg as std_msg
import sensor_msgs.msg as sensor_msgs
import platform_motion_msgs.msg as platform_msg
import platform_motion_srv.srv as platform_srv
import manipulator_msgs.msg as manipulator_msg
import geometry_msgs.msg as geometry_msg
import move_base_msgs.msg as move_base_msg
import visual_servo_msgs.msg as visual_servo_msg
import linemod_detector.msg as linemod_msg
import dynamic_reconfigure.srv as dynsrv
import dynamic_reconfigure.msg as dynmsg
import tf_conversions

from executive import executive_manual
from executive import executive_level_one
from executive import executive_level_two

from executive.monitor_topic_state import MonitorTopicState

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
        self.preemptCV = threading.Condition()
 
        # node parameters
        self.wait_for_cameras = rospy.get_param("~wait_for_cameras", True)
        self.servo_feedback_interval = rospy.Duration(rospy.get_param("~servo_feedback_interval", 5.0))
        self.gpio_servo_id = rospy.get_param("~gpio_servo_id", 1)
        self.pursuit_complete_distance = rospy.get_param("~pursuit_complete_distance", 5.0)
        self.home_pursuit_complete_distance = rospy.get_param("~home_pursuit_complete_distance", 10.0)
        self.maximum_pursuit_error = rospy.get_param("~maximum_pursuit_error", 1.5)
        self.beacon_scan_distance = rospy.get_param("~beacon_scan_distance", 10.0)
            #get pin assignments, they are strings in hex format, convert to ints
        self.GPIO_PIN_LEVEL_ONE = int(rospy.get_param("~GPIO_PIN_LEVEL_ONE", '0x20'), 16)
        self.GPIO_PIN_LEVEL_TWO = int(rospy.get_param("~GPIO_PIN_LEVEL_TWO", '0x08'), 16)
        
        # subscribe to topics
        rospy.Subscriber("gpio_read", platform_msg.GPIO, self.gpio_update)
        rospy.Subscriber("pause_state", std_msg.Bool, self.pause_state_update)
        rospy.Subscriber("search_point",
                         linemod_msg.NamedPoint,
                         self.sample_detection_search_update)
        rospy.Subscriber("manipulator_detection_point",
                         linemod_msg.NamedPoint,
                         self.sample_detection_manip_update)
        rospy.Subscriber("beacon_pose",
                          geometry_msg.PoseStamped,
                          self.beacon_update)

        self.listener = tf.TransformListener()
        
        
        # create publishers
        self.navigation_audio=rospy.Publisher("audio_navigate", platform.msg_VoiceAnnouncement)
        
        self.joystick_command=rospy.Publisher("joystick_command", geometry_msg.Twist)
        self.planner_command=rospy.Publisher("planner_command", geometry_msg.Twist)
        self.servo_command=rospy.Publisher("CAN_servo_command", geometry_msg.Twist)

        '''
        # create service proxies
        self.platform_motion_input_select = \
                rospy.ServiceProxy("CAN_select_command_source",
                platform_srv.SelectCommandSource)

        self.set_planner_parameters = \
                rospy.ServiceProxy('DWAPlanner_set_parameters',
                dynsrv.Reconfigure)

        # action clients
        self.home_wheelpods = \
                actionlib.SimpleActionClient("home_wheel_pods",
                platform_msg.HomeAction)
        
        self.home_carousel = \
                actionlib.SimpleActionClient("home_carousel",
                platform_msg.HomeAction)
   
        self.manipulator = \
                actionlib.SimpleActionClient('manipulator_action',
                manipulator_msg.ManipulatorAction)

        self.move_base = actionlib.SimpleActionClient("planner_move_base",
                move_base_msg.MoveBaseAction)
        
        self.servo = actionlib.SimpleActionClient("visual_servo_action",
                visual_servo_msg.VisualServoAction)

        self.drive_home_task = None
        '''
        
        self.top_state_machine = smach.StateMachine(outcomes=['shutdown', 'aborted'],
                                                    input_keys = [],
                                                    output_keys = [])
        
        
        #Beginning of top level state machine declaration.
        #In this machine, "preempted" will indicate a normal command
        #from another process or node, requesting the state machine to pause
        #and handle some input.  This will probably be primarily for handling
        #mode changes. "aborted" will indicate an internal failure in a state 
        #that prevents the state from executing properly.
        #The "next" outcome will be used for states that have only one possible transition.
        with self.top_state_machine: 
            
            smach.StateMachine.add('START',
                                    StartState(self.wait_for_cameras),
                                    transitions = { 'check_cameras':'WAIT_FOR_CAMERAS',
                                                    'skip_cameras':'HOME_PLATFORM',
                                                    'preempted':'TOP_PREEMPTED',
                                                    'aborted':'TOP_ABORTED'})
                        
            cam_dict = {'NAV_CENTER' : 'camera_started',
                        'NAV_PORT' : 'camera_started',
                        'NAV_STARBOARD' : 'camera_started',
                        'MANIPULATOR' : 'camera_started',
                        'SEARCH' : 'camera_started'}
            
            #concurrency container to allow waiting for cameras in parallel
            wait_for_cams = smach.Concurrence(outcomes = ['all_started',
                                                          'timeout',
                                                          'preempted'],
                                              default_outcome = 'timeout',
                                              input_keys = [],
                                              output_keys = [],
                                              outcome_map = { 'all_started' : cam_dict})
            
            with wait_for_cams:
                
                camera_wait_outcomes = [ ('Ready', 'camera_started', 1 ) ]
                
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
                
                smach.Concurrence.add('MANIPULATOR',
                                      MonitorTopicState('manipulator_camera_status',
                                                        'data',
                                                        std_msg.String,
                                                        camera_wait_outcomes,
                                                        timeout = 10.0))
                
                smach.Concurrence.add('SEARCH',
                                      MonitorTopicState('search_camera_status',
                                                        'data',
                                                        std_msg.String,
                                                        camera_wait_outcomes,
                                                        timeout = 10.0))
                
            smach.StateMachine.add('WAIT_FOR_CAMERAS',
                                   wait_for_cams,
                                   transitions = {'all_started':'HOME_PLATFORM',
                                                  'timeout':'CAMERA_FAILURE',
                                                  'preempted':'TOP_PREEMPTED'})
            
            smach.StateMachine.add('CAMERA_FAILURE',
                                   CameraFailure(),
                                   transitions = {'next':'HOME_PLATFORM'})
            
            smach.StateMachine.add('HOME_PLATFORM',
                                   HomePlatform(),
                                   transitions = {'homed':'CHECK_MODE',
                                                  'preempted':'TOP_PREEMPTED',
                                                  'aborted':'TOP_ABORTED'})
     
            smach.StateMachine.add('CHECK_MODE',
                                   CheckMode(),
                                   transitions = {'manual':'MANUAL_MODE',
                                                  'level_one':'LEVEL_ONE_MODE',
                                                  'level_two':'LEVEL_TWO_MODE',
                                                  'preempted':'TOP_PREEMPTED',
                                                  'aborted':'TOP_ABORTED'})                       

            #create the mode state machines, and pass them to the
            #function which adds their states
            level_one = smach.StateMachine(outcomes=['complete', 'preempted', 'aborted'])
            executive_level_one.AddStates(level_one)
            smach.StateMachine.add('LEVEL_ONE_MODE',
                                   level_one,
                                   transitions = {'complete':'CHECK_MODE',
                                                  'preempted':'CHECK_MODE',
                                                  'aborted':'TOP_ABORTED'})

            level_two = smach.StateMachine(outcomes=['complete', 'preempted', 'aborted'])
            executive_level_two.AddStates(level_two)
            smach.StateMachine.add('LEVEL_TWO_MODE',
                                   level_two,
                                   transitions = {'complete':'CHECK_MODE',
                                                  'preempted':'CHECK_MODE',
                                                  'aborted':'TOP_ABORTED'})

            manual =  smach.StateMachine(outcomes=['complete', 'preempted', 'aborted'])
            executive_manual.AddStates(manual)
            smach.StateMachine.add('MANUAL_MODE',
                                   manual,
                                   transitions = {'complete':'CHECK_MODE',
                                                  'preempted':'CHECK_MODE',
                                                  'aborted':'TOP_ABORTED'})                        

            smach.StateMachine.add('TOP_PREEMPTED',
                                   TopPreempted(),
                                   transitions = {'next':'CHECK_MODE'})
            
            smach.StateMachine.add('TOP_ABORTED',
                                   TopAborted(),
                                   transitions = {'recover':'CHECK_MODE',
                                                  'fail':'aborted'})
            
        sls = smach_ros.IntrospectionServer('top_introspection', self.top_state_machine, '/START')
        sls.start()
        
        self.top_state_machine.execute()

    #topic handlers

    def gpio_update(self, gpio):
        if gpio.servo_id == self.gpio_servo_id:
            #store the new gpio value, and pass it to any functions that should react
            self.gpio = gpio
            self.handle_mode_switch(gpio.new_pin_states)
                
    def sample_detection_search_update(self, msg):
        self.search_sample = msg

    def sample_detection_manip_update(self, msg):
        self.manip_sample = msg
        
    #other methods

    def handle_mode_switch(self, pin_states):
        if pin_states&self.GPIO_PIN_LEVEL_ONE == 0:
            detected_mode == "LEVEL_ONE_MODE"
        elif pin_states&self.GPIO_PIN_LEVEL_TWO == 0:
            detected_mode == "LEVEL_TWO_MODE"
        else:
            detected_mode == "MANUAL_MODE"
        
        #it requires two matches to change mode
        detect_match = detected_mode == self.last_detected_mode
        self.last_detected_mode = detected_mode
                                
        if (detected_mode != self.selected_mode) & detect_match:
            self.selected_mode = detected_mode
            active_state = self.top_state_machine.get_active_states()[0]
            if active_state in self.preempt_modes:
                #if we are in one of the sub state machines, preempt
                self.top_state_machine.request_preempt()
                self.preemptCV.acquire()
                #wait until state machine succesfully preempts the current mode
                self.preemptCV.wait()
                self.preemptCV.release    
                self.top_state_machine.service_preempt()

    def announce(self, utterance):
        msg = SoundRequest()
        msg.sound = SoundRequest.SAY
        msg.command = SoundRequest.PLAY_ONCE
        msg.arg = utterance
        msg.arg2 = 'voice.select "%s"'%self.voice
        self.navigation_audio.publish(msg)
                
#state classes for the top level machine

#first state in top level executive, wait for other nodes to come up here
#no need to start the state machine if other services are not available, etc.
class StartState(smach.State):
    def __init__(self, wait_for_cameras):
        smach.State.__init__(self,
                             outcomes=['check_cameras', 'skip_cameras', 'preempted', 'aborted'])
        self.wait_for_cameras = wait_for_cameras
                
    def execute(self, userdata):
        if self.wait_for_cameras:
            return 'check_cameras'
        else:
            return 'skip_cameras'

#this state homes all mechanisms on the robot platform
class HomePlatform(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            input_keys=['camera_status'],
                            outcomes=['homed', 'aborted','preempted'])   
    
    def execute(self, userdata):
        
        #home servos            
               
        return 'aborted'
    
class CheckMode(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             input_keys=['camera_status'],
                             outcomes=['manual', 'level_one', 'level_two', 'preempted', 'aborted'])
        
    def execute(self, userdata):
        
        return 'succeeded'
    
class CameraFailure(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             output_keys=['camera_status'],
                             outcomes=['next'])
        
    def execute(self, userdata):
        
        userdata.camera_status = 'error'
                
        return 'next'
    
class TopPreempted(smach.State):
    def __init__(self, preemptCV):
        smach.State.__init__(self, outcomes=['next'])
        
        self.preemptCV = preemptCV
        
    def execute(self, userdata):
        
        self.preemptCV.acquire()
        self.preemptCV.notifyAll()
        self.preemptCV.release()
        
        return 'next'

class TopAborted(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['recover','fail'])
        
    def execute(self, userdata):
        
        return 'fail'
    



    