#!/usr/bin/python
# the actual ros node for the manipulator code

import roslib; roslib.load_manifest("manipulator")
import rospy
import actionlib
import smach
import smach_ros

from std_msgs.msg import Float64

from dynamixel_msgs.msg import JointState
from dynamixel_controllers.srv import *

from manipulator.msg import ManipulatorGrabAction, ManipulatorGrabFeedback, ManipulatorGrabResult

from platform_motion.msg import SelectCarouselBinGoal, SelectCarouselBinAction

from manipulator import manipulator_states

from manipulator.srv import VelocityStandoff, VelocityStandoffRequest
from manipulator.srv import TorqueStandoff, TorqueStandoffRequest
from manipulator.srv import TorqueHold, TorqueHoldRequest
from manipulator.srv import GoToPosition, GoToPositionRequest

class ManipulatorStateMachine(object):
    
  def __init__(self):
  
    # load the node's parameters
    self.arm_down_velocity = rospy.get_param('~arm_down_velocity', -0.5)
    self.arm_down_torque = rospy.get_param('~arm_down_torque', 0.5)
    self.arm_down_standoff = rospy.get_param('~arm_down_standoff', 0.1)
    
    self.arm_up_velocity = rospy.get_param('~arm_up_velocity', 0.5)
    self.arm_up_torque = rospy.get_param('~arm_up_torque', 0.6)
    self.arm_up_standoff = rospy.get_param('~arm_up_standoff', 0.1)
    
    self.hand_open_position = rospy.get_param('~hand_open_position', 0.0)
    self.grip_torque = 1
    
    self.wrist_zero_position = rospy.get_param('~wrist_zero_position', 0.0)
    
    # wait for all the dynamixel services to actually come live.
    rospy.loginfo('Waiting for manipulator/wrist_joint services...')
    rospy.wait_for_service('wrist_joint/go_to_position')
  
    rospy.loginfo('Waiting for manipulator/arm_joint services...')
    rospy.wait_for_service('arm_joint/go_to_position')
  
    rospy.loginfo('Waiting for manipulator/hand_joint services...')
    rospy.wait_for_service('hand_joint/go_to_position')
  
    # make service proxies important services
    
    self.arm_velocity_standoff = rospy.ServiceProxy('arm_joint/velocity_standoff', VelocityStandoff)  
    self.arm_go_to_position = rospy.ServiceProxy('arm_joint/go_to_position', GoToPosition)  
    self.wrist_go_to_position = rospy.ServiceProxy('wrist_joint/go_to_position', GoToPosition)  
    self.hand_go_to_position = rospy.ServiceProxy('hand_joint/go_to_position', GoToPosition)
    
    #home the manipulator on startup
    
    self.arm_velocity_standoff(self.arm_up_velocity, self.arm_up_torque, self.arm_up_standoff)
    self.wrist_go_to_position(self.wrist_zero_position)
    self.hand_go_to_position(self.hand_open_position)
         
    # start actually creating the state machine!
    sm = smach.StateMachine(
        outcomes=['success', 'aborted', 'preempted'],
        input_keys = ['goal'],
        output_keys = ['result']
    )
  
    with sm:
      
      smach.StateMachine.add(
          'START',
          manipulator_states.ProcessGoal(),
          transitions = {'succeeded':'ROTATE_WRIST',
                         'invalid_goal':'ERROR'}
      )

      @smach.cb_interface(input_keys=['wrist_angle'])
      def wrist_angle_cb(userdata, request):
        return GoToPositionRequest(userdata.wrist_angle)
  
      smach.StateMachine.add(
          'ROTATE_WRIST',
          smach_ros.ServiceState('wrist_joint/go_to_position', GoToPosition,
          request_cb = wrist_angle_cb,
          input_keys = ['wrist_angle']),          
          transitions = {'succeeded':'ARM_DOWN',
                         'aborted':'ERROR'}
      )

      smach.StateMachine.add(
          'ARM_DOWN',
          smach_ros.ServiceState('arm_joint/velocity_standoff', VelocityStandoff,
          request = VelocityStandoffRequest(self.arm_down_velocity, self.arm_down_torque, self.arm_down_standoff)),          
          transitions = {'succeeded':'GRIP_SAMPLE',
                         'aborted':'ERROR'}
      )
 
      @smach.cb_interface(input_keys=['grip_torque'])
      def grip_torque_cb(userdata, request):
        return TorqueHoldRequest(userdata.grip_torque)   
 
      smach.StateMachine.add(
          'GRIP_SAMPLE',
          smach_ros.ServiceState('hand_joint/torque_hold', TorqueHold,
          request_cb = grip_torque_cb,
          input_keys = ['grip_torque']),          
          transitions = {'succeeded':'ARM_UP',
                         'aborted':'ERROR'}
      )

      smach.StateMachine.add(
          'ARM_UP',
          smach_ros.ServiceState('arm_joint/velocity_standoff', VelocityStandoff,
          request = VelocityStandoffRequest(self.arm_up_velocity, self.arm_up_torque, self.arm_up_standoff)),          
          transitions = {'succeeded':'HOME_WRIST',
                         'aborted':'ERROR'}
      ) 
    
      smach.StateMachine.add(
          'HOME_WRIST',
          smach_ros.ServiceState('wrist_joint/go_to_position', GoToPosition,
          request = GoToPositionRequest(self.wrist_zero_position)),
          transitions = {'succeeded':'GET_BIN',
                         'aborted':'ERROR'}
      ) 
    
      @smach.cb_interface(input_keys=['target_bin'])
      def get_carousel_bin_cb(userdata, request):
        goal = SelectCarouselBinGoal()   
        goal.bin_index = userdata.target_bin
        return goal 
        
      #carousel bin is acquired from datastore via callback method
      smach.StateMachine.add(
          'GET_BIN',
          smach_ros.SimpleActionState('/select_carousel_bin',
            SelectCarouselBinAction,
            goal_cb=get_carousel_bin_cb,
            input_keys = ['target_bin']),
          transitions = {'succeeded':'OPEN_HAND'}
      )
 
      smach.StateMachine.add(
          'OPEN_HAND',
          smach_ros.ServiceState('hand_joint/go_to_position', GoToPosition,
          request = GoToPositionRequest(self.hand_open_position)),
          transitions = {'succeeded':'CLEAR_CAROUSEL',
                         'aborted':'ERROR'}
      )
  
      #return carousel to bin 0
      carouselBin = SelectCarouselBinGoal()
      carouselBin.bin_index = 0
      smach.StateMachine.add(
          'CLEAR_CAROUSEL',
          smach_ros.SimpleActionState('/select_carousel_bin',
            SelectCarouselBinAction,
            goal=carouselBin),
          transitions = {'succeeded':'success'}
      )
   
      smach.StateMachine.add(
          'ERROR',
          manipulator_states.ErrorState(),
          transitions = {'failure':'aborted',
                         'succeeded':'aborted'}
      )

   # now that the state machine is fully defined, make the action server wrapper
    wrapper = smach_ros.ActionServerWrapper(
        'manipulator_grab', ManipulatorGrabAction,
        wrapped_container = sm,
        succeeded_outcomes = ['success'],
        aborted_outcomes = ['aborted'],
        preempted_outcomes = ['preempted'],
        goal_key = 'goal'
    )
  
    sls = smach_ros.IntrospectionServer('smach_grab_introspection', sm, '/START')
    sls.start()
  
    wrapper.run_server()
    
  
if __name__=="__main__":
  # init the ros node
  rospy.init_node("manipulator")
  msm = ManipulatorStateMachine()
  rospy.spin()
