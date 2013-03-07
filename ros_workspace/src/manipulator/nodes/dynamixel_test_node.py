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

def main():
  # init the ros node
  rospy.init_node("dynamixel_test")

  # wait for all the dynamixel services to actually come live.
  rospy.loginfo('Waiting for dynamixel_test_controller services...')
  rospy.wait_for_service('dynamixel_test_controller/set_speed')
  rospy.wait_for_service('dynamixel_test_controller/torque_enable')
  rospy.wait_for_service('dynamixel_test_controller/set_compliance_slope')
  rospy.wait_for_service('dynamixel_test_controller/set_compliance_margin')
  rospy.wait_for_service('dynamixel_test_controller/set_compliance_punch')
  rospy.wait_for_service('dynamixel_test_controller/set_torque_limit')

  rospy.spin()

if __name__=="__main__":
  main()
