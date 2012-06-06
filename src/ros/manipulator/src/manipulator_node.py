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

import manipulator_states
from manipulator_data import PersistantData

def main():
  # init the ros node
  rospy.init_node("manipulator")

  # set up the persistant data for the smach states
  dataStore = PersistantData()

  # get a boatload of parameters from the param server
  dataStore.wristMinPos = rospy.get_param('manipulator/wrist_min_pos')
  dataStore.wristMaxPos = rospy.get_param('manipulator/wrist_max_pos')
  dataStore.wristSpeed = rospy.get_param('manipulator/wrist_speed', 1.0)
  dataStore.wristTorque = rospy.get_param('manipulator/wrist_torque', 0.5)

  dataStore.armDownSpeed = rospy.get_param('manipulator/arm_down_speed', 1.0)
  dataStore.armDownTorque = rospy.get_param('manipulator/arm_down_torque', 0.8)
  dataStore.armUpSpeed = rospy.get_param('manipulator/arm_up_speed', 1.0)
  dataStore.armUpTorque = rospy.get_param('manipulator/arm_up_torque', 0.5)
  dataStore.armHoldTorque = rospy.get_param('manipulator/arm_hold_torque',
      dataStore.armUpTorque
  )

  dataStore.handMinPos = rospy.get_param('manipulator/hand_min_pos')
  dataStore.handMaxPos = rospy.get_param('manipulator/hand_max_pos')
  dataStore.handCloseSpeed = rospy.get_param('manipulator/hand_close_speed', 1.0)
  dataStore.handOpenSpeed = rospy.get_param('manipulator/hand_open_speed',
      dataStore.handCloseSpeed
  )
  dataStore.handCloseTorque = rospy.get_param('manipulator/hand_close_torque', 0.8)
  dataStore.handOpenTorque = rospy.get_param('manipulator/hand_open_torque', 0.5)
  dataStore.handHoldTorque = rospy.get_param('manipulator/hand_hold_torque',
      dataStore.handOpenTorque
  )

  # wait for all the dynamixel services to actually come live.
  rospy.loginfo('Waiting for wristJointController services...')
  rospy.wait_for_service('wristJointController/set_speed')
  rospy.wait_for_service('wristJointController/torque_enable')
  rospy.wait_for_service('wristJointController/set_compliance_slope')
  rospy.wait_for_service('wristJointController/set_compliance_margin')
  rospy.wait_for_service('wristJointController/set_compliance_punch')
  rospy.wait_for_service('wristJointController/set_torque_limit')

  rospy.loginfo('Waiting for armJointController services...')
  rospy.wait_for_service('armJointController/set_speed')
  rospy.wait_for_service('armJointController/torque_enable')
  rospy.wait_for_service('armJointController/set_compliance_slope')
  rospy.wait_for_service('armJointController/set_compliance_margin')
  rospy.wait_for_service('armJointController/set_compliance_punch')
  rospy.wait_for_service('armJointController/set_torque_limit')

  rospy.loginfo('Waiting for handJointController services...')
  rospy.wait_for_service('handJointController/set_speed')
  rospy.wait_for_service('handJointController/torque_enable')
  rospy.wait_for_service('handJointController/set_compliance_slope')
  rospy.wait_for_service('handJointController/set_compliance_margin')
  rospy.wait_for_service('handJointController/set_compliance_punch')
  rospy.wait_for_service('handJointController/set_torque_limit')

  # make service proxies for all of the services and give them to the dataStore
  # it will likely only ever use them to actually initialize things and
  # possibly do some error recovery.
  dataStore.wristSetSpeedService = rospy.ServiceProxy(
      'wristJointController/set_speed',
      SetSpeed
  )
  dataStore.wristSetTorqueEnableService = rospy.ServiceProxy(
      'wristJointController/torque_enable',
      TorqueEnable
  )
  dataStore.wristSetComplianceSlopeService = rospy.ServiceProxy(
      'wristJointController/set_compliance_slope',
      SetComplianceSlope
  )
  dataStore.wristSetComplianceMarginService = rospy.ServiceProxy(
      'wristJointController/set_compliance_margin',
      SetComplianceMargin
  )
  dataStore.wristSetCompliancePunchService = rospy.ServiceProxy(
      'wristJointController/set_compliance_punch',
      SetCompliancePunch
  )
  dataStore.wristSetTorqueLimitService = rospy.ServiceProxy(
      'wristJointController/set_torque_limit',
      SetTorqueLimit
  )

  dataStore.armSetSpeedService = rospy.ServiceProxy(
      'armJointController/set_speed',
      SetSpeed
  )
  dataStore.armSetTorqueEnableService = rospy.ServiceProxy(
      'armJointController/torque_enable',
      TorqueEnable
  )
  dataStore.armSetComplianceSlopeService = rospy.ServiceProxy(
      'armJointController/set_compliance_slope',
      SetComplianceSlope
  )
  dataStore.armSetComplianceMarginService = rospy.ServiceProxy(
      'armJointController/set_compliance_margin',
      SetComplianceMargin
  )
  dataStore.armSetCompliancePunchService = rospy.ServiceProxy(
      'armJointController/set_compliance_punch',
      SetCompliancePunch
  )
  dataStore.armSetTorqueLimitService = rospy.ServiceProxy(
      'armJointController/set_torque_limit',
      SetTorqueLimit
  )

  dataStore.handSetSpeedService = rospy.ServiceProxy(
      'handJointController/set_speed',
      SetSpeed
  )
  dataStore.handSetTorqueEnableService = rospy.ServiceProxy(
      'handJointController/torque_enable',
      TorqueEnable
  )
  dataStore.handSetComplianceSlopeService = rospy.ServiceProxy(
      'handJointController/set_compliance_slope',
      SetComplianceSlope
  )
  dataStore.handSetComplianceMarginService = rospy.ServiceProxy(
      'handJointController/set_compliance_margin',
      SetComplianceMargin
  )
  dataStore.handSetCompliancePunchService = rospy.ServiceProxy(
      'handJointController/set_compliance_punch',
      SetCompliancePunch
  )
  dataStore.handSetTorqueLimitService = rospy.ServiceProxy(
      'handJointController/set_torque_limit',
      SetTorqueLimit
  )

  # set the dataStore as the subscriber to the joint state controller state
  # messages.
  rospy.Subscriber(
      'wristJointController/state',
      JointState,
      dataStore.WristJointCallback
  )

  rospy.Subscriber(
      'armJointController/state',
      JointState,
      dataStore.ArmJointCallback
  )

  rospy.Subscriber(
      'handJointController/state',
      JointState,
      dataStore.HandJointCallback
  )

  # make publishers for the dataStore as well
  dataStore.wristPosPublisher = rospy.Publisher(
      'wristJointController/command',
      Float64
  )

  dataStore.armJointPosPublisher = rospy.Publisher(
      'armJointController/command',
      Float64
  )

  dataStore.handJointPosPublisher = rospy.Publisher(
      'handJointController/command',
      Float64
  )

  # start actually creating the state machine!
  sm = smach.StateMachine(
      outcomes=['success', 'aborted', 'preempted'],
      input_keys = ['goal'],
      output_keys = ['result']
  )

  with sm:
    smach.StateMachine.add(
        'START',
        manipulator_states.HandleGoal(dataStore),
        transitions = {'success':'ROTATE_WRIST'}
    )

    smach.StateMachine.add(
        'ROTATE_WRIST',
        manipulator_states.RotateWrist(dataStore),
        transitions = {'success':'SETUP_ARM_SPEED_DOWN',
                       'failure':'ERROR'}
    )

    
    smach.StateMachine.add(
      'SETUP_ARM_SPEED_DOWN',
      smach_ros.ServiceState('/armJointController/set_speed',
        SetSpeed,
        request = SetSpeedRequest(speed=dataStore.armDownSpeed)),
      transitions={'succeeded':'SETUP_ARM_TORQUE_DOWN'}
    )

    
    smach.StateMachine.add(
      'SETUP_ARM_TORQUE_DOWN',
      smach_ros.ServiceState('armJointController/set_torque_limit',
        SetTorqueLimit,
        request = SetTorqueLimitRequest(abs(dataStore.armDownTorque))),
      transitions = {'succeeded':'SETUP_ARM_TORQUE_ENABLE_DOWN'}
    )

    smach.StateMachine.add(
        'SETUP_ARM_TORQUE_ENABLE_DOWN',
        smach_ros.ServiceState('armJointController/torque_enable',
          TorqueEnable,
          request = TorqueEnableRequest(True)),
        transitions = {'succeeded':'START_MOVING_ARM_DOWN'}
    )

    smach.StateMachine.add(
        'START_MOVING_ARM_DOWN',
        manipulator_states.StartMovingArm(dataStore),
        transitions = {'success':'WAIT_FOR_ARM_STOP_DOWN',
                       'failure':'ERROR'}
    )

    smach.StateMachine.add(
        'WAIT_FOR_ARM_STOP_DOWN',
        manipulator_states.WaitForArmStop(dataStore, dataStore.armDownTorque),
        transitions = {'success':'SETUP_ARM_HOLD_DOWN',
                       'failure':'ERROR'}
    )

    smach.StateMachine.add(
      'SETUP_ARM_HOLD_DOWN',
      smach_ros.ServiceState('armJointController/set_torque_limit',
        SetTorqueLimit,
        request = SetTorqueLimitRequest( abs(dataStore.armHoldTorque) )
        ),
      transitions = {'succeeded':'SETUP_HAND_SPEED_CLOSE'}
    )

    smach.StateMachine.add(
        'SETUP_HAND_SPEED_CLOSE',
      smach_ros.ServiceState('handJointController/set_speed',
        SetSpeed,
        request = SetSpeedRequest( dataStore.handCloseSpeed )),
      transitions={'succeeded':'SETUP_HAND_TORQUE_CLOSE'}
    )

    smach.StateMachine.add(
      'SETUP_HAND_TORQUE_CLOSE',
      smach_ros.ServiceState('handJointController/set_torque_limit',
        SetTorqueLimit,
        request = SetTorqueLimitRequest( abs(dataStore.handCloseTorque) )
        ),
      transitions = {'succeeded':'SETUP_HAND_TORQUE_ENABLE_CLOSE'}
    )

    smach.StateMachine.add(
        'SETUP_HAND_TORQUE_ENABLE_CLOSE',
        smach_ros.ServiceState('handJointController/torque_enable',
          TorqueEnable,
          request = TorqueEnableRequest(True)),
        transitions = {'succeeded':'START_MOVING_HAND_CLOSE'}
    )

    smach.StateMachine.add(
        'START_MOVING_HAND_CLOSE',
        manipulator_states.StartMovingHand(dataStore),
        transitions = {'success':'WAIT_FOR_HAND_CLOSE',
                       'failure':'ERROR'}
    )

    smach.StateMachine.add(
        'WAIT_FOR_HAND_CLOSE',
        manipulator_states.WaitForHandStop(dataStore,
          dataStore.handCloseTorque),
        transitions = {'success':'SETUP_HAND_HOLD_CLOSE',
                       'failure':'ERROR'}
    )

    smach.StateMachine.add(
      'SETUP_HAND_HOLD_CLOSE',
      smach_ros.ServiceState('handJointController/set_torque_limit',
        SetTorqueLimit,
        request = SetTorqueLimitRequest( abs(dataStore.handHoldTorque) )
        ),
      transitions = {'succeeded':'SETUP_ARM_SPEED_UP'}
    )

    smach.StateMachine.add(
      'SETUP_ARM_SPEED_UP',
      smach_ros.ServiceState('armJointController/set_speed',
        SetSpeed,
        request = SetSpeedRequest( dataStore.armUpSpeed )),
      transitions={'succeeded':'SETUP_ARM_TORQUE_UP'}
    )

    smach.StateMachine.add(
      'SETUP_ARM_TORQUE_UP',
      smach_ros.ServiceState('armJointController/set_torque_limit',
        SetTorqueLimit,
        request = SetTorqueLimitRequest( abs(dataStore.armUpTorque) )
        ),
      transitions = {'succeeded':'SETUP_ARM_TORQUE_ENABLE_UP'}
    )

    smach.StateMachine.add(
        'SETUP_ARM_TORQUE_ENABLE_UP',
        smach_ros.ServiceState('armJointController/torque_enable',
          TorqueEnable,
          request = TorqueEnableRequest(True)),
        transitions = {'succeeded':'START_MOVING_ARM_UP'}
    )

    smach.StateMachine.add(
        'START_MOVING_ARM_UP',
        manipulator_states.StartMovingArm(dataStore),
        transitions = {'success':'WAIT_FOR_ARM_STOP_UP',
                       'failure':'ERROR'}
    )

    smach.StateMachine.add(
        'WAIT_FOR_ARM_STOP_UP',
        manipulator_states.WaitForArmStop(dataStore, dataStore.armUpTorque),
        transitions = {'success':'SETUP_ARM_HOLD_UP',
                       'failure':'ERROR'}
    )

    smach.StateMachine.add(
      'SETUP_ARM_HOLD_UP',
      smach_ros.ServiceState('armJointController/set_torque_limit',
        SetTorqueLimit,
        request = SetTorqueLimitRequest( abs(dataStore.armHoldTorque) )
        ),
      transitions = {'succeeded':'GET_BIN'}
    )

    # XXX TODO: use a different way to get a bin!
    carouselBin = SelectCarouselBinGoal()
    carouselBin.bin_index = 5
    smach.StateMachine.add(
        'GET_BIN',
        smach_ros.SimpleActionState('select_carousel_bin',
          SelectCarouselBinAction,
          goal=carouselBin),
        transitions = {'succeeded':'SETUP_HAND_SPEED_OPEN'}
    )

    smach.StateMachine.add(
        'SETUP_HAND_SPEED_OPEN',
      smach_ros.ServiceState('handJointController/set_speed',
        SetSpeed,
        request = SetSpeedRequest( dataStore.handOpenSpeed ) ),
      transitions={'succeeded':'SETUP_HAND_TORQUE_OPEN'}
    )

    smach.StateMachine.add(
      'SETUP_HAND_TORQUE_OPEN',
      smach_ros.ServiceState('handJointController/set_torque_limit',
        SetTorqueLimit,
        request = SetTorqueLimitRequest( abs(dataStore.handOpenTorque) )
        ),
      transitions = {'succeeded':'SETUP_HAND_TORQUE_ENABLE_OPEN'}
    )

    
    smach.StateMachine.add(
        'SETUP_HAND_TORQUE_ENABLE_OPEN',
        smach_ros.ServiceState('handJointController/torque_enable',
          TorqueEnable,
          request = TorqueEnableRequest(True)),
        transitions = {'succeeded':'START_MOVING_HAND_OPEN'}
    )

    smach.StateMachine.add(
        'START_MOVING_HAND_OPEN',
        manipulator_states.StartMovingHand(dataStore),
        transitions = {'success':'WAIT_FOR_HAND_OPEN',
                       'failure':'ERROR'}
    )

    smach.StateMachine.add(
        'WAIT_FOR_HAND_OPEN',
        manipulator_states.WaitForHandStop(dataStore,
          dataStore.handOpenTorque),
        transitions = {'success':'SETUP_HAND_LIMP',
                       'failure':'ERROR'}
    )

    # disable hand torque enable after the hand has opened
    smach.StateMachine.add(
        'SETUP_HAND_LIMP',
        smach_ros.ServiceState('handJointController/torque_enable',
          TorqueEnable,
          request = TorqueEnableRequest(False)),
        transitions = {'succeeded':'CLEAR_CAROUSEL'}
    )

    # XXX TODO: use a different way to get a bin!
    carouselBin = SelectCarouselBinGoal()
    carouselBin.bin_index = 0
    smach.StateMachine.add(
        'CLEAR_CAROUSEL',
        smach_ros.SimpleActionState('select_carousel_bin',
          SelectCarouselBinAction,
          goal=carouselBin),
        transitions = {'succeeded':'SETUP_ARM_LIMP'}
    )

    # turn off arm torque enable to make the arm go limp after carousel is
    # clear
    smach.StateMachine.add(
        'SETUP_ARM_LIMP',
        smach_ros.ServiceState('armJointController/torque_enable',
          TorqueEnable,
          request = TorqueEnableRequest(False)),
        transitions = {'succeeded':'success'}
    )

    smach.StateMachine.add(
        'ERROR',
        manipulator_states.ErrorState(dataStore),
        transitions = {'failure':'aborted'}
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

  # according to the docs, run_server is non blocking, so spin is needed.
  wrapper.run_server()

  rospy.spin()


if __name__=="__main__":
  main()
