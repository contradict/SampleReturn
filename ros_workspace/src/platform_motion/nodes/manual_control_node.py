import smach
import smach_ros
import rospy
import actionlib
import collections

import manipulator_msgs.msg as manipulator_msg
import platform_motion_msgs.msg as platform_msg
import platform_motion_msgs.srv as platform_srv
import visual_servo_msgs.msg as visual_servo_msg
import geometry_msgs.msg as geometry_msg
import sensor_msgs.msg as sensor_msgs
import samplereturn.util as util


#the top level executive passes the level one state machine object to this
#function.  This is where its states are added

class ManualController(object):
    
    def __init__(self):
        
        #stuff namedtuple with joystick parameters
        self.node_params = util.get_node_params()
        
        CAN_source_select = \
                rospy.ServiceProxy("CAN_select_command_source",
                platform_srv.SelectCommandSource)
    
        self.sm = smach.StateMachine(
                  outcomes=['complete', 'preempted', 'aborted'],
                  input_keys = ['action_goal'],
                  output_keys = ['action_result'])
    
        
        with sm:
            
            smach.StateMachine.add('PROCESS_GOAL',
                                   ProcessGoal(),
                                   transitions = {'valid_goal':'JOYSTICK_LISTEN',
                                                 'invalid_goal':'MANUAL_ABORTED'})
            
            smach.StateMachine.add('JOYSTICK_LISTEN',
                                   JoystickListen(CAN_source_select, self.node_params),
                                   transitions = {'visual_servo_requested':'VISUAL_SERVO',
                                                  'manipulator_grab_requested':'MANIPULATOR_GRAB',
                                                  'preempted':'MANUAL_PREEMPTED',
                                                  'aborted':'MANUAL_ABORTED'})           
                
             
            smach.StateMachine.add('VISUAL_SERVO',
                                    VisualServo(CAN_source_select),
                                    transitions = {'complete':'JOYSTICK_DRIVING',
                                                   'preempted':'MANUAL_PREEMPTED',
                                                   'aborted':'MANUAL_ABORTED'})
             
            smach.StateMachine.add('MANIPULATOR_GRAB',
                                    ManipulatorGrab(CAN_source_select),
                                    transitions = {'complete':'JOYSTICK_DRIVING',
                                                   'preempted':'MANUAL_PREEMPTED',
                                                   'aborted':'MANUAL_ABORTED'})
             
            smach.StateMachine.add('MANUAL_PREEMPTED',
                                     ManualPreempted(CAN_source_select),
                                     transitions = {'complete':'preempted',
                                                   'fail':'aborted'})
             
            smach.StateMachine.add('MANUAL_ABORTED',
                                    ManualAborted(CAN_source_select),
                                    transitions = {'recover':'JOYSTICK_DRIVING',
                                                   'fail':'aborted'})
            
            #end with sm     
  
        #action server wrapper    
        manual_control_server = smach_ros.ActionServerWrapper(
            'manual_control', platform_msg.ManualControlAction,
            wrapped_container = self.sm,
            succeeded_outcomes = ['complete'],
            preempted_outcomes = ['preempted'],
            aborted_outcomes = ['aborted'],
            goal_key = 'action_goal',
            result_key = 'action_result')
        
        sls = smach_ros.IntrospectionServer('smach_grab_introspection', self.sm, '/PROCESS_GOAL')
        sls.start()
        
        #start action servers and services
        manual_control_server.run_server()
    

class ProcessGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['valid_goal',
                                       'invalid_goal'],
                             input_keys=['action_goal'],
                             output_keys=['action_feedback',
                                          'allow_driving',
                                          'allow_manipulator'])
            
    def execute(self, userdata):
        
        fb = platform_msg.ManualControlFeedback()
        fb.state = "PROCESS_GOAL"

        mcg = platform_msg.ManualControlGoal()
        
        userdata.allow_driving = False
        userdata.allow_manipulator = False
        
        if userdata.action_goal.mode == mcg.FULL_CONTROL:
            userdata.allow_driving = True
            userdata.allow_manipulator = True
        elif userdata.action_goal.mode == mcg.DRIVING_ONLY:
            user_data.allow_driving = True
        elif userdata.action_goal.mode == mcg.MANIPULATOR_ONLY:
            userdata.allow_manipulator = True
        else:
            return 'invalid_goal'
                        
        return 'valid_goal'


class JoystickListen(smach.State):
    def __init__(self, CAN_source_select, node_params):
        smach.State.__init__(self,
                             outcomes=['visual_servo_requested',
                                       'manipulator_grab_requested',
                                       'preempted',
                                       'aborted'],
                             input_keys=['action_goal', 'allow_driving', 'allow_manipulator'],
                             output_keys=['action_feedback'])
        
        self.CAN_source_select = CAN_source_select
        self.joy_params = node_params
        
    def execute(self, userdata):
        
        self.allow_driving = userdata.allow_driving
        self.allow_manipulator = userdata.allow_manipulator
        
        servo_command=rospy.Publisher("joystick_command", geometry_msg.Twist)
        self.CAN_source_select("Joystick")
        servo_command.publish((0,0,0),(0,0,0))
        joy_sub = rospy.Subscriber("joy", sensor_msg.Joy ,self.joy_callback)  
        
        
        
        
        servo_command.shutdown()
        
        return 'sample_detected'
    
    def joy_callback(self, joy_msg):
        
        if self.allow_manipulator:
            if joy_msg.buttons[0]:
                #start visual servo
            if joy_msg.buttons[0]:
                #start grab acton
        
        if self.allow_driving:
        

#drive to detected sample location        
class VisualServo(smach.State):
    def __init__(self, CAN_source_select):
        smach.State.__init__(self,
                             outcomes=['complete', 'preempted', 'aborted'])
        
        self.CAN_source_select = CAN_source_select
        
    def execute(self, userdata):
    
        visual_servo_action = actionlib.SimpleActionClient("visual_servo_action",
                              visual_servo_msg.VisualServoAction)
        
    
        return 'complete'
    
class ManipulatorGrab(smach.State):
    def __init__(self, CAN_source_select):
        smach.State.__init__(self,
                             outcomes=['complete', 'preempted', 'aborted'])
        
        self.CAN_source_select = CAN_source_select
        
    def execute(self, userdata):
        
        manipulator_action = \
        actionlib.SimpleActionClient('manipulator_action',
        manipulator_msg.ManipulatorAction)
        
        return 'complete'
    
class ManualPreempted(smach.State):
    def __init__(self, CAN_source_select):
        smach.State.__init__(self, outcomes=['complete','fail'])
        
        self.CAN_source_select = CAN_source_select
        
    def execute(self):
        
        return 'complete'

class ManualAborted(smach.State):
    def __init__(self, CAN_source_select):
        smach.State.__init__(self, outcomes=['recover','fail'])
        
        self.CAN_source_select = CAN_source_select
        
    def execute(self):
        
        return 'fail'
    
    