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

from executive import executive_states

class ExecutiveMaster(object):
    
    def __init__(self):
        
        # ordinary variables
        self.last_servo_feedback = None
        self.lost_sample = None

        '''
        # node parameters
        self.voice = rospy.get_param("~voice", "kal_diphone")
        self.speech_delay = rospy.get_param("~speech_delay", 0.8)
        self.speech_rate = rospy.get_param("~speech_rate", 0.07)
        self.servo_feedback_interval =\
            rospy.Duration(rospy.get_param("~servo_feedback_interval", 5.0))
        self.gpio_servo_id = rospy.get_param("~gpio_servo_id", 1)
        self.pursuit_complete_distance = rospy.get_param("~pursuit_complete_distance", 5.0)
        self.home_pursuit_complete_distance = rospy.get_param("~home_pursuit_complete_distance", 10.0)
        self.maximum_pursuit_error = rospy.get_param("~maximum_pursuit_error", 1.5)
        self.beacon_scan_distance = rospy.get_param("~beacon_scan_distance", 10.0)
        self.wait_for_cameras = rospy.get_param("~wait_for_cameras", True)

        # subscribe to interesting topics
        rospy.Subscriber("gpio_read", platform_msg.GPIO, self.gpio_update)
        rospy.Subscriber("navigation_camera_status", std_msg.String,
                self.navigation_status_update)
        rospy.Subscriber("manipulator_camera_status", std_msg.String,
                self.manipulator_status_update)
        rospy.Subscriber("pause_state", std_msg.Bool,
                self.pause_state_update)
        rospy.Subscriber("search_point",
                linemod_msg.NamedPoint,
                self.sample_detection_search_update)
        rospy.Subscriber("manipulator_detection_point", linemod_msg.NamedPoint,
                self.sample_detection_man_update)
        rospy.Subscriber("beacon_pose", geometry_msg.PoseStamped,
                self.beacon_update)

        self.listener = tf.TransformListener()

        # create publishers
        self.navigation_audio=rospy.Publisher("audio_navigate", SoundRequest)
        self.joystick_command=rospy.Publisher("joystick_command", geometry_msg.Twist)
        self.planner_command=rospy.Publisher("planner_command", geometry_msg.Twist)
        self.servo_command=rospy.Publisher("CAN_servo_command", geometry_msg.Twist)

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
        
        self.top_state_machine = smach.StateMachine(
            outcomes=['success', 'preempted', 'aborted'],
            input_keys = ['action_goal'],
            output_keys = ['action_result']
        )
        
        
        #Beginning of top level state machine declaration.
        #In this machine, "preempted" will indicate a normal command
        #from another process or node, requesting the state machine to pause
        #and handle some input.  This will probably be primarily for handling
        #mode changes.
        #"aborted" will indicate an internal failure in a state that prevents
        #the state from executing properly
        with self.top_state_machine: 
            
            smach.StateMachine.add(
                'START',
                StartState(),
                transitions = { 'check_cameras':'WAIT_FOR_CAMERAS',
                                'skip_cameras':'HOME_PLATFORM',
                                'preempted':'TOP_PREEMPTED',
                                'aborted':'TOP_ABORTED'}
            )
            
            cam_dict = {'NAV_CENTER' : 'started',
                        'NAV_PORT' : 'started',
                        'NAV_STARBOARD' : 'started',
                        'MANIPULATOR' : 'started',
                        'SEARCH' : 'started'}
            
            #concurrency container to allow waiting for cameras in parallel
            wait_for_cams = smach.Concurrence(outcomes = ['all_started',
                                                          'timeout',
                                                          'preempted',
                                                          'aborted'],
                                              default_outcome = 'timeout',
                                              input_keys = [],
                                              output_keys = [],
                                              outcome_map = { 'succeeded' : cam_dict}
            )
            
            with wait_for_cams:
                
                smach.Concurrence.add('NAV_CENTER',
                                      WaitForCamera('nav_center_topic'),
                                      )
                                
                smach.Concurrence.add('NAV_PORT',
                                      WaitForCamera('nav_port_topic'),
                                      )
                
                smach.Concurrence.add('NAV_STARBOARD',
                                      WaitForCamera('nav_starboard_topic')
                                     )
                
                smach.Concurrence.add('MANIPULATOR',
                                      WaitForCamera('manipulator_topic')
                                     )
                
                smach.Concurrence.add('SEARCH',
                                      WaitForCamera('search_topic')
                                     )
                
            smach.StateMachine.add('WAIT_FOR_CAMERAS',
                                   wait_for_cams,
                                   transitions = {'succeeded':'HOME_PLATFORM',
                                                  'timeout':'CAMERA_FAILURE',
                                                  'preempted':'TOP_PREEMPTED',
                                                  'aborted' : 'TOP_ABORTED'}
                                  )
            
            
            smach.StateMachine.add('HOME_PLATFORM',
                                   HomePlatform(),
                                   transitions = {'homed':'MODE_CHECK',
                                                  'timeout':'TOP_ABORTED',
                                                  'preempted':'TOP_PREEMPTED',
                                                  'aborted':'TOP_ABORTED'}
                                   )
            
            
        sls = smach_ros.IntrospectionServer('top_introspection', self.top_state_machine, '/START')
        sls.start()

    def start(self):
        self.top_state_machine.execute()
        
        
#state classes for the top level machine

#first state in top level executive, wait for other nodes to come up here
#no need to start the state machine if other services are not available, etc.
class StartState(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             input_keys=['wait_for_cameras'],
                             outcomes=['check_cameras', 'skip_cameras', 'preempted', 'aborted']
        )
        
    def execute(self, userdata):
        if userdata.wait_for_cameras:
            return 'check_cameras'
        else:
            return 'skip_cameras'

#this state waits for a camera (or pair) to properly start        
class WaitForCamera(smach.State):
    def __init__(self, camera_topic):
        smach.State.__init__(self, outcomes=['camera_started', 'timeout'])
        self.camera_topic = camera_topic
                    
    def execute(self, userdata):

        return 'camera_started'

#this state homes all mechanisms on the robot platform
class HomePlatform(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            input_keys=['mode'],
                            outcomes=['homed','aborted','preempted']
        )   
    
    def execute(self, userdata):
        
        #home servos            
               
        return 'homed'
    
class CheckMode(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[])
        
    def execute(self):
        
        return 'succeeded'
    
class TopPreempted(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[])
        
    def execute(self):
        
        return 'succeeded'

class TopAborted(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[])
        
    def execute(self):
        
        return 'succeeded'
    



    