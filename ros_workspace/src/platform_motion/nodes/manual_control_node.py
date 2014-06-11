#!/usr/bin/env python
#manual control node
import math
import smach
import smach_ros
import rospy
import collections
import threading
import actionlib
import tf

import actionlib_msgs.msg as action_msg
import manipulator_msgs.msg as manipulator_msg
import platform_motion_msgs.msg as platform_msg
import platform_motion_msgs.srv as platform_srv
import visual_servo_msgs.msg as visual_servo_msg
import geometry_msgs.msg as geometry_msg
import sensor_msgs.msg as sensor_msg
import samplereturn_msgs.msg as samplereturn_msg
import samplereturn.util as util
import samplereturn.simple_motion as simple_motion

from samplereturn_msgs.msg import VoiceAnnouncement

from executive.executive_states import SelectMotionMode
from executive.executive_states import AnnounceState
from executive.executive_states import ExecuteSimpleMove
from executive.executive_states import ServoController

#this state machine provides manual control of the robot

class ManualController(object):
    
    def __init__(self):
        
        rospy.on_shutdown(self.shutdown_cb)
        
        #stuff namedtuple with joystick parameters
        self.node_params = util.get_node_params()
        self.joy_state = JoyState(self.node_params)
        self.CAN_interface = util.CANInterface()
        self.announcer = util.AnnouncerInterface("audio_navigate")
        self.tf = tf.TransformListener()
 
        #get a simple_mover, it's parameters are inside a rosparam tag for this node
        self.simple_mover = simple_motion.SimpleMover('~simple_move_params/', self.tf)
    
        self.state_machine = smach.StateMachine(
                  outcomes=['complete', 'preempted', 'aborted'],
                  input_keys = ['action_goal'],
                  output_keys = ['action_result'])
        
        self.state_machine.userdata.button_cancel = self.joy_state.button('BUTTON_CANCEL')
        self.state_machine.userdata.detected_sample = None
        self.state_machine.userdata.paused = False
        
        # disable obstacle checking in ExecuteSimpleMove
        self.state_machine.userdata.active_strafe_key = None

        #strafe search settings
        self.state_machine.userdata.settle_time = 1
        #set move tolerance huge, this prevent retrying by the simple mover
        self.state_machine.userdata.simple_move_tolerance = 1.0
        self.state_machine.userdata.manipulator_correction = self.node_params.manipulator_correction
        self.state_machine.userdata.servo_params = self.node_params.servo_params
        self.state_machine.userdata.velocity = None
        
        #use these as booleans in remaps
        self.state_machine.userdata.true = True
        self.state_machine.userdata.false = False
        
        with self.state_machine:
            
            MODE_JOYSTICK = platform_srv.SelectMotionModeRequest.MODE_JOYSTICK
            MODE_SERVO = platform_srv.SelectMotionModeRequest.MODE_SERVO
            MODE_PAUSE = platform_srv.SelectMotionModeRequest.MODE_PAUSE
            MODE_RESUME = platform_srv.SelectMotionModeRequest.MODE_RESUME
            MODE_HOME = platform_srv.SelectMotionModeRequest.MODE_HOME
            MODE_UNLOCK = platform_srv.SelectMotionModeRequest.MODE_UNLOCK
            MODE_LOCK = platform_srv.SelectMotionModeRequest.MODE_LOCK
           
            smach.StateMachine.add('START_MANUAL_CONTROL',
                                   ProcessGoal(self.announcer),
                                   transitions = {'valid_goal':'SELECT_JOYSTICK',
                                                 'invalid_goal':'MANUAL_ABORTED'})
            
            smach.StateMachine.add('SELECT_JOYSTICK',
                                   SelectMotionMode(self.CAN_interface,
                                                    MODE_JOYSTICK),
                                   transitions = {'next':'JOYSTICK_LISTEN',
                                                  'failed':'MANUAL_ABORTED'})
            
            smach.StateMachine.add('JOYSTICK_LISTEN',
                                   JoystickListen(self.CAN_interface, self.joy_state),
                                   transitions = {'visual_servo_requested':'SELECT_SERVO',
                                                  'manipulator_grab_requested':'PAUSE_FOR_GRAB',
                                                  'home_wheelpods_requested':'SELECT_HOME',
                                                  'lock_wheelpods_requested':'SELECT_PAUSE_FOR_LOCK',
                                                  'preempted':'MANUAL_PREEMPTED',
                                                  'aborted':'MANUAL_ABORTED'})           
            
            smach.StateMachine.add('SELECT_SERVO',
                                   SelectMotionMode(self.CAN_interface,
                                                    MODE_SERVO),
                                   transitions = {'next':'VISUAL_SERVO',
                                                  'failed':'SELECT_JOYSTICK'})

            #calculate the strafe move to the sample
            smach.StateMachine.add('VISUAL_SERVO',
                                   ServoController(self.tf, self.announcer),
                                   transitions = {'move':'SERVO_MOVE',
                                                  'complete':'ANNOUNCE_SERVO_COMPLETE',
                                                  'point_lost':'ANNOUNCE_NO_SAMPLE',
                                                  'aborted':'ANNOUNCE_FAILURE'})

            smach.StateMachine.add('SERVO_MOVE',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'VISUAL_SERVO',
                                                  'timeout':'VISUAL_SERVO',
                                                  'aborted':'ANNOUNCE_FAILURE',
                                                  })

            smach.StateMachine.add('ANNOUNCE_FAILURE',
                                   AnnounceState(self.announcer,
                                                 'Visual servo failure'),
                                   transitions = {'next':'SELECT_JOYSTICK'})

            smach.StateMachine.add('ANNOUNCE_NO_SAMPLE',
                                   AnnounceState(self.announcer,
                                                 'No sample in view, cancel in'),
                                   transitions = {'next':'SELECT_JOYSTICK'})
            
            smach.StateMachine.add('ANNOUNCE_SERVO_COMPLETE',
                                   AnnounceState(self.announcer,
                                                 'Servo complete'),
                                   transitions = {'next':'SELECT_JOYSTICK'})   
            
            smach.StateMachine.add('ANNOUNCE_SERVO_CANCELED',
                                   AnnounceState(self.announcer,
                                                 'Servo canceled'),
                                   transitions = {'next':'SELECT_JOYSTICK'})   
            
            smach.StateMachine.add('PAUSE_FOR_GRAB',
                                   SelectMotionMode(self.CAN_interface,
                                                    MODE_PAUSE),
                                   transitions = {'next':'MANIPULATOR_GRAB',
                                                  'failed':'SELECT_JOYSTICK'})
 
            def grab_msg_cb(userdata):
                grab_msg = manipulator_msg.ManipulatorGoal()
                grab_msg.type = grab_msg.GRAB
                grab_msg.grip_torque = 0.7
                grab_msg.target_bin = 1
                grab_msg.wrist_angle = 0
                if userdata.detected_sample is not None:
                    grab_msg.wrist_angle = userdata.detected_sample.grip_angle
                    grab_msg.target_bin = userdata.detected_sample.sample_id
                return grab_msg
            
            smach.StateMachine.add('MANIPULATOR_GRAB',
                                   InterruptibleActionClientState(
                                       'manipulator_action',
                                       manipulator_msg.ManipulatorAction,
                                       None,
                                       grab_msg_cb,
                                       None,
                                       self.announcer,
                                       "Manipulator unavailable",
                                       "Grabbing",
                                       30.0,
                                       "Manipulator grab timed out"),
                                    transitions = {'complete':'ANNOUNCE_GRAB_COMPLETE',
                                                   'canceled':'ANNOUNCE_GRAB_CANCELED',
                                                   'preempted':'MANUAL_PREEMPTED',
                                                   'aborted':'MANUAL_ABORTED'})

            smach.StateMachine.add('ANNOUNCE_GRAB_COMPLETE',
                                   AnnounceState(self.announcer,
                                                 'Grab complete'),
                                   transitions = {'next':'UNPAUSE_AFTER_GRAB'})   
            
            smach.StateMachine.add('ANNOUNCE_GRAB_CANCELED',
                                   AnnounceState(self.announcer,
                                                 'Servo canceled'),
                                   transitions = {'next':'UNPAUSE_AFTER_GRAB'})
            
            smach.StateMachine.add('UNPAUSE_AFTER_GRAB',
                                   SelectMotionMode(self.CAN_interface,
                                                    MODE_RESUME),
                                   transitions = {'next':'SELECT_JOYSTICK',
                                                  'failed':'MANUAL_ABORTED'})

            smach.StateMachine.add('MANUAL_PREEMPTED',
                                     ManualPreempted(self.CAN_interface),
                                     transitions = {'complete':'preempted',
                                                   'fail':'aborted'})
             
            smach.StateMachine.add('MANUAL_ABORTED',
                                    ManualAborted(self.CAN_interface),
                                    transitions = {'recover':'SELECT_JOYSTICK',
                                                   'fail':'aborted'})

            smach.StateMachine.add('SELECT_HOME',
                                   SelectMotionMode(self.CAN_interface,
                                                    MODE_HOME),
                                   transitions = {'next':'PERFORM_HOME',
                                                  'failed':'SELECT_JOYSTICK'})

            home_goal = platform_msg.HomeGoal()
            home_goal.home_count = 3
            smach.StateMachine.add('PERFORM_HOME',
                                   InterruptibleActionClientState(
                                       'wheel_pods/home',
                                       platform_msg.HomeAction,
                                       home_goal,
                                       None,
                                       None,
                                       self.announcer,
                                       "Home ing unavailable",
                                       "Home ing",
                                       30.0,
                                       "Home ing timed out"),
                                    transitions = {'complete':'SELECT_JOYSTICK',
                                                   'canceled':'SELECT_JOYSTICK',
                                                   'preempted':'MANUAL_PREEMPTED',
                                                   'aborted':'MANUAL_ABORTED'})

            smach.StateMachine.add('SELECT_PAUSE_FOR_LOCK',
                                   SelectMotionMode(self.CAN_interface,
                                                    MODE_PAUSE),
                                   transitions = {'next':'SELECT_LOCK',
                                                  'failed':'SELECT_LOCK'})

            smach.StateMachine.add('SELECT_LOCK',
                                   SelectMotionMode(self.CAN_interface,
                                                    MODE_LOCK),
                                   transitions = {'next':'ANNOUNCE_LOCK',
                                                  'failed':'RESUME_FROM_LOCK'})

            smach.StateMachine.add('ANNOUNCE_LOCK',
                                   AnnounceState(self.announcer,
                                                 'Wheel lock selected'),
                                   transitions = {'next':'WAIT_FOR_UNLOCK'})

            smach.StateMachine.add('WAIT_FOR_UNLOCK',
                                    WaitForJoystickButton(self.joy_state,
                                                          'BUTTON_LOCK',
                                                          self.announcer),
                                    transitions = {'pressed':'SELECT_UNLOCK',
                                                   'timeout':'SELECT_LOCK',
                                                   'preempted':'MANUAL_PREEMPTED'})

            smach.StateMachine.add('SELECT_UNLOCK',
                                   SelectMotionMode(self.CAN_interface,
                                                    MODE_UNLOCK),
                                   transitions = {'next':'RESUME_FROM_LOCK',
                                                  'failed':'RESUME_FROM_LOCK'})

            smach.StateMachine.add('RESUME_FROM_LOCK',
                                   SelectMotionMode(self.CAN_interface,
                                                    MODE_RESUME),
                                   transitions = {'next':'JOYSTICK_LISTEN',
                                                  'failed':'SELECT_JOYSTICK'})

             #end with state_machine


        #action server wrapper    
        manual_control_server = smach_ros.ActionServerWrapper(
            'manual_control', platform_msg.ManualControlAction,
            wrapped_container = self.state_machine,
            succeeded_outcomes = ['complete'],
            preempted_outcomes = ['preempted'],
            aborted_outcomes = ['aborted'],
            goal_key = 'action_goal',
            result_key = 'action_result')
        
        sls = smach_ros.IntrospectionServer('smach_grab_introspection',
                                            self.state_machine,
                                            '/START_MANUAL_CONTROL')
        sls.start()


        joy_sub = rospy.Subscriber("joy", sensor_msg.Joy, self.joy_callback)
        self.sample_sub_manipulator = rospy.Subscriber('detected_sample_manipulator',
                                                        samplereturn_msg.NamedPoint,
                                                        self.sample_detection_manipulator)
        
        #start action servers and services
        manual_control_server.run_server()

    def joy_callback(self, joy_msg):
        #store message and current time in joy_state
        self.joy_state.update(joy_msg)
        self.state_machine.userdata.button_cancel = self.joy_state.button('BUTTON_CANCEL')
        
    def sample_detection_manipulator(self, sample):
        self.state_machine.userdata.detected_sample = sample

    def shutdown_cb(self):
        self.state_machine.request_preempt()
        while self.state_machine.is_running():
            rospy.sleep(0.1)
   
class ProcessGoal(smach.State):
    def __init__(self, announcer):
        smach.State.__init__(self,
                             outcomes=['valid_goal',
                                       'invalid_goal'],
                             input_keys=['action_goal'],
                             output_keys=['action_result',
                                          'action_feedback',
                                          'allow_driving',
                                          'allow_manipulator'])
        
        self.announcer = announcer
            
    def execute(self, userdata):
        
        fb = platform_msg.ManualControlFeedback()
        fb.state = "PROCESS_GOAL"
        result = platform_msg.ManualControlResult('initialized')
        userdata.action_result = result        

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

class JoystickListen(smach.State):
    def __init__(self, CAN_interface, joy_state):
        smach.State.__init__(self,
                             outcomes=['visual_servo_requested',
                                       'manipulator_grab_requested',
                                       'home_wheelpods_requested',
                                       'lock_wheelpods_requested',
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


    def set_outcome(self, outcome):
        self.button_outcome = outcome
        self.button_CV.acquire()
        self.button_CV.notifyAll()
        self.button_CV.release()

    def driving_callback(self, event):
        
        if self.preempt_requested():
            rospy.loginfo("MANUAL CONTROL PREEMPTED")
            self.service_preempt()
            self.set_outcome('preempted')
            return
        
        if self.allow_manipulator:
            if self.joy_state.button('BUTTON_SERVO'):
                self.set_outcome( 'visual_servo_requested' )
                return
            if self.joy_state.button('BUTTON_GRAB'):
                self.set_outcome( 'manipulator_grab_requested' )
                return

        
        if self.allow_driving:
            if self.joy_state.button('BUTTON_HOME'):
                self.set_outcome( 'home_wheelpods_requested' )
                return
            if self.joy_state.button('BUTTON_LOCK'):
                self.set_outcome( 'lock_wheelpods_requested' )
                return
            self.CAN_interface.publish_joy_state(self.joy_state)

class WaitForJoystickButton(smach.State):
    def __init__(self, joy_state, button, announcer, timeout=None):
        smach.State.__init__(self,
                             outcomes=['pressed',
                                       'timeout',
                                       'preempted',
                                       ],
                             )

        self.joy_state = joy_state

        self.button = button

        self.announcer = announcer

        self.timeout = timeout

    def execute(self, userdata):
        button_state = self.joy_state.button( self.button )

        debounce_count = 5

        debounce = debounce_count

        outcome = None

        timeout = self.timeout

        dt = 0.1

        rate = rospy.Rate( 1./dt )
        while outcome == None:
            rate.sleep()
            new_button_state =  self.joy_state.button( self.button )
            if new_button_state and not button_state:
                debounce -= 1
                if debounce == 0:
                    outcome = 'pressed'
            else:
                debounce = debounce_count
                button_state = new_button_state
            if self.preempt_requested():
                self.outcome = 'preempted'
            if timeout is not None:
                timeout -= dt
                if timeout <= 0:
                    outcome = 'timeout'

        self.announcer.say( "Release to unlock." )

        while self.joy_state.button(self.button):
            rate.sleep()

        return outcome


class InterruptibleActionClientState(smach.State):
    def __init__(self, actionname, actionspec,
            goal = None,
            goal_cb = None,
            feedback_cb = None,
            announcer=None,
            unavailable_msg=None,
            active_msg=None,
            timeout=None,
            timeout_msg=None,
            ):
        smach.State.__init__(self,
                             input_keys=['button_cancel', 'detected_sample'],
                             outcomes=['complete', 'canceled', 'preempted', 'aborted'])

        self.actionname = actionname
        self.actionspec = actionspec
        self.goal = goal
        self.goal_cb = goal_cb
        self.feedback_cb = feedback_cb
        self.announcer = announcer
        self.unavailable_msg = unavailable_msg
        self.active_msg = active_msg
        self.timeout = timeout
        self.timeout_msg = timeout_msg

    def execute(self, userdata):
        action_client = actionlib.SimpleActionClient(self.actionname,
                self.actionspec)

        if not action_client.wait_for_server(timeout=rospy.Duration(1.0)) :
            if self.unavailable_msg is not None:
                self.announcer.say( self.unavailable_msg )
            return 'aborted'

        #if goal_cb is defined, it overwrites goal!
        if (self.goal_cb is not None):
            self.goal = self.goal_cb(userdata)

        action_client.send_goal( self.goal,
                done_cb = self.goal_done,
                active_cb = self.goal_active,
                feedback_cb = self.goal_feedback
                )

        self.outcome = None

        def timeoutcancel(evt):
            action_client.cancel_all_goals()
            if self.timeout_msg is not None:
                self.announcer.say( self.timeout_msg )
            self.outcome = 'canceled'

        if self.timeout is not None:
            timer = rospy.Timer( rospy.Duration( self.timeout ),
                    timeoutcancel,
                    oneshot=True )

        rate = rospy.Rate( 0.1 )
        while self.outcome == None:
            if userdata.button_cancel:
                action_client.cancel_all_goals()
                self.outcome = 'canceled'
            if self.preempt_requested():
                action_client.cancel_all_goals()
                self.outcome = 'preempted'
            rate.sleep()

        timer.shutdown()

        return self.outcome

    def goal_active(self):
        if self.active_msg is not None:
            self.announcer.say( self.active_msg )


    def goal_done(self, state, result):
        if state == action_msg.GoalStatus.SUCCEEDED:
            self.outcome = 'complete'
        if state == action_msg.GoalStatus.PREEMPTED:
            self.outcome = 'canceled'
        if state == action_msg.GoalStatus.ABORTED:
            self.outcome = 'aborted'
        
    def goal_feedback(self, feedback):
        if self.feedback_cb is not None:
            result = self.feedback_cb(feedback, self.announcer)
            if result is not None:
                self.outcome = result


class ManualPreempted(smach.State):
    def __init__(self, CAN_interface):
        smach.State.__init__(self, outcomes=['complete','fail'],
                                   output_keys=['action_result'])
        
        self.CAN_interface = CAN_interface
        
    def execute(self, userdata):
        
        #we are preempted by the top state machine
        #set motion mode to None and exit
        self.CAN_interface.select_mode(platform_srv.SelectMotionModeRequest.MODE_PAUSE)

        self.CAN_interface.select_mode(platform_srv.SelectMotionModeRequest.MODE_ENABLE)
        
        result = platform_msg.ManualControlResult('preempted')
        userdata.action_result = result
        
        return 'complete'

class ManualAborted(smach.State):
    def __init__(self, CAN_interface):
        smach.State.__init__(self, outcomes=['recover','fail'],
                                   output_keys=['action_result'])
        
        self.CAN_interface = CAN_interface
        
    def execute(self, userdata):
        result = platform_msg.ManualControlResult('aborted')
        userdata.action_result = result
        
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
        #rospy.loginfo("MANUAL CONTROL, msg.buttons = " + str(self.msg.buttons))
        #rospy.loginfo("MANUAL CONTROL, button index = " + str(getattr(self.joy_params, button_name)))

        button_index = getattr(self.joy_params, button_name)
        
        if button_index >= len(self.msg.buttons):
            return False #always return false if the requested button is not in the list
        else:
            return bool(self.msg.buttons[button_index])
    
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
        
if __name__ == '__main__':
    rospy.init_node("manual_control_node")
    mcn = ManualController()
    rospy.spin()
    
