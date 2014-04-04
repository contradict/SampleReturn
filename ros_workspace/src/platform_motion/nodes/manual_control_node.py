#!/usr/bin/python
#manual control node

import math
import smach
import smach_ros
import rospy
import actionlib
import collections
import threading

import manipulator_msgs.msg as manipulator_msg
import platform_motion_msgs.msg as platform_msg
import platform_motion_msgs.srv as platform_srv
import visual_servo_msgs.msg as visual_servo_msg
import geometry_msgs.msg as geometry_msg
import sensor_msgs.msg as sensor_msg
import samplereturn.util as util

from samplereturn_msgs.msg import VoiceAnnouncement

#this state machine provides manual control of the robot

class ManualController(object):
    
    def __init__(self):
        
        #stuff namedtuple with joystick parameters
        self.node_params = util.get_node_params()
        self.joy_state = JoyState(self.node_params)
        self.CAN_interface = CANInterface()
        self.announcer = util.AnnouncerInterface("audio_navigate")
            
        joy_sub = rospy.Subscriber("joy", sensor_msg.Joy, self.joy_callback)
    
        self.sm = smach.StateMachine(
                  outcomes=['complete', 'preempted', 'aborted'],
                  input_keys = ['action_goal'],
                  output_keys = ['action_result'])
        
        with self.sm:
            
           
            smach.StateMachine.add('START_MANUAL_CONTROL',
                                   ProcessGoal(self.announcer),
                                   transitions = {'valid_goal':'SELECT_JOYSTICK',
                                                 'invalid_goal':'MANUAL_ABORTED'})
            
            smach.StateMachine.add('SELECT_JOYSTICK',
                                   SelectMotionMode(self.CAN_interface,
                                       platform_srv.SelectMotionModeRequest.MODE_JOYSTICK),
                                   transitions = {'next':'JOYSTICK_LISTEN',
                                                 'aborted':'MANUAL_ABORTED'})
            
            smach.StateMachine.add('JOYSTICK_LISTEN',
                                   JoystickListen(self.CAN_interface, self.joy_state),
                                   transitions = {'visual_servo_requested':'SELECT_SERVO',
                                                  'manipulator_grab_requested':'SELECT_PAUSE',
                                                  'preempted':'MANUAL_PREEMPTED',
                                                  'aborted':'MANUAL_ABORTED'})           
            
            smach.StateMachine.add('SELECT_SERVO',
                                   SelectMotionMode(self.CAN_interface,
                                       platform_srv.SelectMotionModeRequest.MODE_SERVO),
                                   transitions = {'next':'VISUAL_SERVO',
                                                  'aborted':'MANUAL_ABORTED'})
             
            smach.StateMachine.add('VISUAL_SERVO',
                                    VisualServo(self.CAN_interface, self.joy_state),
                                    transitions = {'complete':'SELECT_JOYSTICK',
                                                   'preempted':'MANUAL_PREEMPTED',
                                                   'aborted':'MANUAL_ABORTED'})
            
            smach.StateMachine.add('SELECT_PAUSE',
                                   SelectMotionMode(self.CAN_interface,
                                       platform_srv.SelectMotionModeRequest.MODE_PAUSE),
                                   transitions = {'next':'MANIPULATOR_GRAB',
                                                  'aborted':'MANUAL_ABORTED'})
             
            smach.StateMachine.add('MANIPULATOR_GRAB',
                                    ManipulatorGrab(self.CAN_interface, self.joy_state),
                                    transitions = {'complete':'SELECT_JOYSTICK',
                                                   'canceled':'SELECT_JOYSTICK',
                                                   'preempted':'MANUAL_PREEMPTED',
                                                   'aborted':'MANUAL_ABORTED'})
             
            smach.StateMachine.add('MANUAL_PREEMPTED',
                                     ManualPreempted(self.CAN_interface),
                                     transitions = {'complete':'preempted',
                                                   'fail':'aborted'})
             
            smach.StateMachine.add('MANUAL_ABORTED',
                                    ManualAborted(self.CAN_interface),
                                    transitions = {'recover':'SELECT_JOYSTICK',
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
        
        sls = smach_ros.IntrospectionServer('smach_grab_introspection',
                                            self.sm,
                                            '/START_MANUAL_CONTROL')
        sls.start()
        
        #start action servers and services
        manual_control_server.run_server()

    def joy_callback(self, joy_msg):
        #store message and current time in joy_state
        self.joy_state.update(joy_msg)
   
class ProcessGoal(smach.State):
    def __init__(self, announcer):
        smach.State.__init__(self,
                             outcomes=['valid_goal',
                                       'invalid_goal'],
                             input_keys=['action_goal'],
                             output_keys=['action_feedback',
                                          'allow_driving',
                                          'allow_manipulator'])
        
        self.announcer = announcer
            
    def execute(self, userdata):
        
        fb = platform_msg.ManualControlFeedback()
        fb.state = "PROCESS_GOAL"

        self.announcer.say("Entering manual mode.")
                        
        userdata.allow_driving = False
        userdata.allow_manipulator = False
        
        mcg = platform_msg.ManualControlGoal()
        if userdata.action_goal.mode == mcg.FULL_CONTROL:
            userdata.allow_driving = True
            userdata.allow_manipulator = True
            self.announcer.say("Full control enabled.")
        elif userdata.action_goal.mode == mcg.DRIVING_ONLY:
            user_data.allow_driving = True
            self.announcer.say("Driving enabled.")
        elif userdata.action_goal.mode == mcg.MANIPULATOR_ONLY:
            userdata.allow_manipulator = True
            self.announcer.say("Manipulator enabled.")
        else:
            return 'invalid_goal'
                        
        return 'valid_goal'

class SelectMotionMode(smach.State):
    def __init__(self, CAN_interface, motion_mode):
        smach.State.__init__(self, outcomes = ['next', 'aborted'])
        self.CAN_interface = CAN_interface
        self.motion_mode = motion_mode
                      
    def execute(self, userdata):
        self.CAN_interface.select_mode(self.motion_mode)
        self.CAN_interface.publish_zero()
        return 'next'
                
class JoystickListen(smach.State):
    def __init__(self, CAN_interface, joy_state):
        smach.State.__init__(self,
                             outcomes=['visual_servo_requested',
                                       'manipulator_grab_requested',
                                       'preempted',
                                       'aborted'],
                             input_keys=['action_goal', 'allow_driving', 'allow_manipulator'],
                             output_keys=['action_feedback'])
        
        self.CAN_interface = CAN_interface
        self.joy_state = joy_state
        self.button_CV = threading.Condition()
        
    def execute(self, userdata):
    
        self.allow_driving = userdata.allow_driving
        self.allow_manipulator = userdata.allow_manipulator
        self.button_outcome = None
 
        #publish the joystick defined twist every 50ms    
        driving_timer = rospy.Timer(rospy.Duration(.05), self.driving_callback)
        
        #wait here, driving, until another button is pressed
        self.button_CV.acquire()
        self.button_CV.wait()
        self.button_CV.release()
        
        driving_timer.shutdown()       
                
        return self.button_outcome
    
    def driving_callback(self, event):
        
        if self.preempt_requested():
            self.service_preempt()
            self.callback_outcome = 'preempted'
            return
        
        if self.allow_manipulator:
            if self.joy_state.button('BUTTON_SERVO'):
                self.button_outcome = 'visual_servo_requested'
                self.button_CV.acquire()
                self.button_CV.notifyAll()
                self.button_CV.release()
                return
            if self.joy_state.button('BUTTON_GRAB'):
                self.button_outcome = 'manipulator_grab_requested'
                self.button_CV.acquire()
                self.button_CV.notifyAll()
                self.button_CV.release()
                #stop robot before using manipulator
                self.CAN_interface.publish_zero()
                return
        
        if self.allow_driving:
            self.CAN_interface.publish_joy_state(self.joy_state)
           
        
#drive to detected sample location        
class VisualServo(smach.State):
    def __init__(self, CAN_interface, joy_state):
        smach.State.__init__(self,
                             outcomes=['complete', 'preempted', 'aborted'])
        
        self.CAN_interface = CAN_interface
        
    def execute(self, userdata):
    
        visual_servo = actionlib.SimpleActionClient("visual_servo_action",
                                                    visual_servo_msg.VisualServoAction)
        
    
        return 'complete'
    
class ManipulatorGrab(smach.State):
    def __init__(self, CAN_interface, joy_state):
        smach.State.__init__(self,
                             outcomes=['complete', 'canceled', 'preempted', 'aborted'])
        
        self.CAN_interface = CAN_interface
        self.joy_state = joy_state
        
    def execute(self, userdata):
        
        working_states = [action_msg.GoalStatus.ACTIVE, action_msg.GoalStatus.PENDING]
               
        manipulator = \
        actionlib.SimpleActionClient('grab_action',
        manipulator_msg.ManipulatorAction)
        
        grab_msg = manipulator_msg.ManipulatorGoal()
        grab_msg.type = grab_msg.GRAB
        grab_msg.grip_torque = 0.7
        grab_msg.target_bin = 2
        
        manipulator.send_goal(grab_msg)
        
        while True:
            state = manipulator.get_state()
            if state == action_msg.GoalStatus.SUCCEEDED:
                return 'complete'
            if state == action_msg.GoalStatus.PREEMPTED:
                return 'preempted'
            if state == action_msg.GoalStatus.ABORTED:
                return 'aborted'
            if joy_state.button('BUTTON_CANCEL'):
                self.manipulator.cancel_all_goals()
                return 'canceled'
            rospy.sleep(0.1)
        
        return 'complete'
    
class ManualPreempted(smach.State):
    def __init__(self, CAN_interface):
        smach.State.__init__(self, outcomes=['complete','fail'])
        
        self.CAN_interface = CAN_interface
        
    def execute(self):
        
        #we are preempted by the top state machine
        #set motion mode to None and exit
        self.CAN_interface.select_mode(
                platform_srv.SelectMotionModeRequest.MODE_PAUSE )
        self.CAN_interface.publish_zero()
        
        return 'complete'

class ManualAborted(smach.State):
    def __init__(self, CAN_interface):
        smach.State.__init__(self, outcomes=['recover','fail'])
        
        self.CAN_interface = CAN_interface
        
    def execute(self):
        
        return 'fail'

#classes to handle joystick and CAN interfacing
    
class JoyState(object):
    def __init__(self, node_params):
        self.msg = sensor_msg.Joy()
        self.msg.axes = [0,0,0,0,0,0]
        self.timestamp = rospy.get_time()
        self.joy_params = node_params
    
    def update(self, joy_msg):
        self.msg = joy_msg
        self.timestamp = rospy.get_time()
        
    def button(self, button_name):
        rospy.loginfo("MANUAL CONTROL, msg.buttons = " + str(self.msg.buttons))
        rospy.loginfo("MANUAL CONTROL, button index = " + str(getattr(self.joy_params, button_name)))

        button_index = getattr(self.joy_params, button_name)
        
        if button_index >= len(self.msg.buttons):
            return False #always return false if the requested button is not in the list
        else:
            return self.msg.buttons[button_index]
    
    def scale_axis(self, scale, exponent, value):
        return math.copysign( scale*(abs(value)**exponent), value)
    
    def get_twist(self):
        twist = geometry_msg.Twist()
        twist.linear.x = self.scale_axis(self.joy_params.LINEAR_SCALE,
                                         self.joy_params.LINEAR_EXP,
                                         self.msg.axes[self.joy_params.LINEAR_X])
        twist.linear.y = self.scale_axis(self.joy_params.LINEAR_SCALE,
                                         self.joy_params.LINEAR_EXP,
                                         self.msg.axes[self.joy_params.LINEAR_Y])
        twist.angular.z = self.scale_axis(self.joy_params.ANGULAR_SCALE,
                                          self.joy_params.ANGULAR_EXP,
                                          self.msg.axes[self.joy_params.ANGULAR_Z])
        
        return twist

class CANInterface(object):
    def __init__(self):
        self.CAN_select_motion_mode = \
                rospy.ServiceProxy("CAN_select_motion_mode",
                platform_srv.SelectMotionMode)
        self.joystick_command=rospy.Publisher("joystick_command", geometry_msg.Twist)
        self.planner_command=rospy.Publisher("planner_command", geometry_msg.Twist)
        self.servo_command=rospy.Publisher("CAN_servo_command", geometry_msg.Twist)

    def select_mode(self, motion_mode):
        self.CAN_select_motion_mode(motion_mode)
        
    def publish_joy_state(self, joy_state):
        self.joystick_command.publish(joy_state.get_twist())
        
    def publish_zero(self):
        t=geometry_msg.Twist()
        t.linear.x=0
        t.linear.y=0
        t.linear.z=0
        t.angular.x=0
        t.angular.y=0
        t.angular.z=0        
        self.joystick_command.publish(t)
        self.planner_command.publish(t)
        self.servo_command.publish(t)
        
if __name__ == '__main__':
    rospy.init_node("manual_control_node")
    mcn = ManualController()
    rospy.spin()
    
