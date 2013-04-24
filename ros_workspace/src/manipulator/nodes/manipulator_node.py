#!/usr/bin/python
# the actual ros node for the manipulator code

import roslib; roslib.load_manifest("manipulator")
import rospy
import actionlib
import smach
import smach_ros
import threading
import time

from std_msgs.msg import Float64
from std_msgs.msg import Bool

from dynamixel_msgs.msg import JointState
from dynamixel_controllers.srv import *

from manipulator.msg import ManipulatorGrabAction, ManipulatorGrabFeedback, ManipulatorGrabResult

from platform_motion.msg import SelectCarouselBinGoal, SelectCarouselBinAction
from platform_motion.srv import Enable, EnableRequest
from platform_motion.msg import HomeAction, HomeGoal, HomeFeedback, HomeResult

from manipulator import manipulator_states

from manipulator.srv import VelocityStandoff, VelocityStandoffRequest
from manipulator.srv import TorqueStandoff, TorqueStandoffRequest
from manipulator.srv import TorqueHold, TorqueHoldRequest
from manipulator.srv import GoToPosition, GoToPositionRequest

class ManipulatorStateMachine(object):
  
  _homeManipulatorFeedback = HomeFeedback()
  _homeManipulatorResult = HomeResult()
    
  def __init__(self):
  
    self.paused = False
    self.pauseCV = threading.Condition()
    
    # load the node's parameters
    self.arm_down_velocity = rospy.get_param('~arm_down_velocity', -0.5)
    self.arm_down_torque = rospy.get_param('~arm_down_torque', 0.5)
    self.arm_down_standoff = rospy.get_param('~arm_down_standoff', 0.1)
    
    self.arm_up_velocity = rospy.get_param('~arm_up_velocity', 0.5)
    self.arm_up_torque = rospy.get_param('~arm_up_torque', 0.6)
    self.arm_up_standoff = rospy.get_param('~arm_up_standoff', 0.1)
    
    self.hand_open_position = rospy.get_param('~hand_open_position', 0.0)
    self.grip_torque = 0.5
    
    self.wrist_zero_position = rospy.get_param('~wrist_zero_position', 0.0)
    
    # wait for all the dynamixel services to actually come live.
    rospy.loginfo('Waiting for manipulator/wrist_joint services...')
    rospy.wait_for_service('wrist_joint/go_to_position')
  
    rospy.loginfo('Waiting for manipulator/arm_joint services...')
    rospy.wait_for_service('arm_joint/go_to_position')
  
    rospy.loginfo('Waiting for manipulator/hand_joint services...')
    rospy.wait_for_service('hand_joint/go_to_position')
  
    self.carousel_client = actionlib.SimpleActionClient('/carousel/select_carousel_bin', SelectCarouselBinAction)    
    
    # make service proxies important services
    
    self.arm_velocity_standoff = rospy.ServiceProxy('arm_joint/velocity_standoff', VelocityStandoff)  
    self.wrist_go_to_position = rospy.ServiceProxy('wrist_joint/go_to_position', GoToPosition)  
    self.hand_go_to_position = rospy.ServiceProxy('hand_joint/go_to_position', GoToPosition)  
    self.arm_pause = rospy.ServiceProxy('arm_joint/pause', Enable)
    self.wrist_pause = rospy.ServiceProxy('wrist_joint/pause', Enable)
    self.hand_pause = rospy.ServiceProxy('hand_joint/pause', Enable)
            
    #advertise the pause service and home action
    self.pause_service = rospy.Service('pause', Enable, self.service_pause)
    self.home_action = actionlib.SimpleActionServer('home', HomeAction, auto_start=False)
    self.home_action.register_goal_callback(self.home_execute)
    self.home_action.register_preempt_callback(self.home_preempt)
    self.home_action.start()
    
    # start actually creating the state machine!
    self.sm = smach.StateMachine(
        outcomes=['success', 'aborted', 'preempted'],
        input_keys = ['goal', 'action_result'],
        output_keys = ['action_result']
    )
  
    with self.sm:
           
      smach.StateMachine.add('START',
          manipulator_states.ProcessGoal(),
          transitions = {'succeeded': 'ROTATE_WRIST',
                         'aborted':'ERROR'}
      )

      @smach.cb_interface()
      def manipulator_response_cb(userdata, response):
        return response.result
      
      @smach.cb_interface(input_keys=['wrist_angle'])
      def wrist_angle_cb(userdata, request):
        return GoToPositionRequest(userdata.wrist_angle)
  
      smach.StateMachine.add('ROTATE_WRIST',
          smach_ros.ServiceState('wrist_joint/go_to_position', GoToPosition,
          request_cb = wrist_angle_cb,
          response_cb = manipulator_response_cb,
          input_keys = ['wrist_angle']),          
          transitions = {'succeeded':'ARM_DOWN',
                         'preempted':'PAUSED',
                         'aborted':'ERROR'}
      )

      smach.StateMachine.add('ARM_DOWN',
          smach_ros.ServiceState('arm_joint/velocity_standoff', VelocityStandoff,
          request = VelocityStandoffRequest(self.arm_down_velocity, self.arm_down_torque, self.arm_down_standoff),
          response_cb = manipulator_response_cb),
          transitions = {'succeeded':'GRIP_SAMPLE',
                         'preempted':'PAUSED',
                         'aborted':'ERROR'}
      )
 
      @smach.cb_interface(input_keys=['grip_torque'])
      def grip_torque_cb(userdata, request):
        return TorqueHoldRequest(-1 * userdata.grip_torque)   
 
      smach.StateMachine.add('GRIP_SAMPLE',
          smach_ros.ServiceState('hand_joint/torque_hold', TorqueHold,
          request_cb = grip_torque_cb,
          response_cb = manipulator_response_cb,
          input_keys = ['grip_torque']),          
          transitions = {'succeeded':'ARM_UP',
                         'preempted':'PAUSED',
                         'aborted':'ERROR'}
      )

      smach.StateMachine.add('ARM_UP',
          smach_ros.ServiceState('arm_joint/velocity_standoff', VelocityStandoff,
          request = VelocityStandoffRequest(self.arm_up_velocity, self.arm_up_torque, self.arm_up_standoff),
          response_cb = manipulator_response_cb),          
          transitions = {'succeeded':'ZERO_WRIST',
                         'preempted':'PAUSED',
                         'aborted':'ERROR'}
      ) 
    
      smach.StateMachine.add('ZERO_WRIST',
          smach_ros.ServiceState('wrist_joint/go_to_position', GoToPosition,
          request = GoToPositionRequest(self.wrist_zero_position),
          response_cb = manipulator_response_cb),
          transitions = {'succeeded':'GET_BIN',
                         'preempted':'PAUSED',
                         'aborted':'ERROR'}
      ) 
    
      @smach.cb_interface(input_keys=['target_bin'])
      def get_carousel_bin_cb(userdata, request):
        goal = SelectCarouselBinGoal()   
        goal.bin_index = userdata.target_bin
        return goal 
        
      #carousel bin is acquired from datastore via callback method
      smach.StateMachine.add('GET_BIN',
          smach_ros.SimpleActionState('/carousel/select_carousel_bin',
          SelectCarouselBinAction,
          goal_cb=get_carousel_bin_cb,
          input_keys = ['target_bin']),
          transitions = {'succeeded':'OPEN_HAND',
                         'preempted':'PAUSED',
                         'aborted':'ERROR'}
      )
 
      smach.StateMachine.add('OPEN_HAND',
          smach_ros.ServiceState('hand_joint/go_to_position', GoToPosition,
          request = GoToPositionRequest(self.hand_open_position),
          response_cb = manipulator_response_cb),          
          transitions = {'succeeded':'CLEAR_CAROUSEL',
                         'preempted':'PAUSED',
                         'aborted':'ERROR'}
      )
  
      #return carousel to bin 0
      #this is always the last state before a successful state machine execution
      carousel_clear_bin = SelectCarouselBinGoal()
      carousel_clear_bin.bin_index = 0
      smach.StateMachine.add('CLEAR_CAROUSEL',
          smach_ros.SimpleActionState('/carousel/select_carousel_bin',
          SelectCarouselBinAction,
          goal=carousel_clear_bin,
          output_keys=['action_result']),
          transitions = {'succeeded':'success',
                         'preempted':'PAUSED',
                         'aborted':'ERROR'}
      )
      
      smach.StateMachine.add('PAUSED',
          manipulator_states.PausedState(self.pauseCV),
          transitions ={'aborted':'aborted',
                        'succeeded':'HOME_ARM'}
      )

      smach.StateMachine.add('HOME_ARM',
          smach_ros.ServiceState('arm_joint/velocity_standoff', VelocityStandoff,
          request = VelocityStandoffRequest(self.arm_up_velocity, self.arm_up_torque, self.arm_up_standoff),
          response_cb = manipulator_response_cb,
          input_keys = ['error','goal']),
          transitions = {'succeeded':'HOME_WRIST',
                         'preempted':'PAUSED',
                         'aborted':'ERROR'}
      ) 

      smach.StateMachine.add('HOME_WRIST',
          smach_ros.ServiceState('wrist_joint/go_to_position', GoToPosition,
          request = GoToPositionRequest(self.wrist_zero_position),
          response_cb = manipulator_response_cb),
          transitions = {'succeeded':'HOME_HAND',
                         'preempted':'PAUSED',
                         'aborted':'ERROR'}
      )

      smach.StateMachine.add('HOME_HAND',
          smach_ros.ServiceState('hand_joint/go_to_position', GoToPosition,
          request = GoToPositionRequest(self.hand_open_position),
          response_cb = manipulator_response_cb),          
          transitions = {'succeeded':'CLEAR_CAROUSEL',
                         'preempted':'PAUSED',
                         'aborted':'ERROR'}
      )

      smach.StateMachine.add('ERROR',
          manipulator_states.ErrorState(),
          transitions = {'error':'aborted'}
      )      

    #now that the state machine is fully defined, make the action server wrapper
    wrapper = smach_ros.ActionServerWrapper(
        'manipulator_grab', ManipulatorGrabAction,
        wrapped_container = self.sm,
        succeeded_outcomes = ['success'],
        aborted_outcomes = ['aborted'],
        preempted_outcomes = ['preempted'],
        goal_key = 'goal',
        result_key = 'action_result'
    )
  
    sls = smach_ros.IntrospectionServer('smach_grab_introspection', self.sm, '/START')
    sls.start()
  
    wrapper.run_server()
    
  def home_execute(self):
    self.home_action.accept_new_goal()
    mgr = ManipulatorGrabResult() #setup the state machine userdata 
    mgr.result = 'homed'
    temp_userdata = smach.UserData()
    temp_userdata['goal'] =  'home'
    temp_userdata['error'] = None
    temp_userdata['action_result'] = mgr
    self.sm.set_initial_state(['HOME_ARM'])
    outcome = self.sm.execute(temp_userdata) #call the state machine with temp_userdata
    self.sm.set_initial_state(['START'])
    if outcome == 'success':
      self._homeManipulatorResult.homed = [True] #homed is a tuple/list
      self.home_action.set_succeeded(result=self._homeManipulatorResult)
    else:
      self._homeManipulatorResult.homed = [False] #homed is a tuple/list
      self.home_action.set_aborted(result=self._homeManipulatorResult)

  def home_preempt(self):
    self._homeManipulatorResult.homed = False
    self.home_action.set_preempted(result=self._homeManipulatorResult)
     
  def service_pause(self, req):
    pause = req.state
    if (pause != self.paused):
      self.paused = pause
      if (pause):
        self.arm_pause(True)
        self.wrist_pause(True)
        self.hand_pause(True)
        self.sm.request_preempt()
        self.carousel_client.cancel_all_goals()
      else:
        self.sm.service_preempt()
        self.arm_pause(False)
        self.wrist_pause(False)
        self.hand_pause(False)
        self.pauseCV.acquire()
        self.pauseCV.notifyAll()
        self.pauseCV.release()              
        
    return self.paused
  
if __name__=="__main__":
  # init the ros node
  rospy.init_node("manipulator")
  msm = ManipulatorStateMachine()
  rospy.spin()
