#!/usr/bin/python

import roslib; roslib.load_manifest("manipulator")
import rospy
import actionlib

from manipulator.msg import ManipulatorGrabAction, ManipulatorGrabGoal, ManipulatorGrabFeedback, ManipulatorGrabResult

from sensor_msgs.msg import Joy

buttonGrab = rospy.get_param('~button_grab', 5)

manipulatorClient = actionlib.SimpleActionClient(
  'manipulator_grab',
  ManipulatorGrabAction
)

def JoystickCallback(msg):
  # when the right button is pushed, emit a manipulator grab command!
  if msg.buttons[buttonGrab] == 1:
    goal = ManipulatorGrabGoal()
    goal.wrist_angle = 0.5
    manipulatorClient.send_goal(goal)
    manipulatorClient.wait_for_result()

def main():
  rospy.init_node("manipulator_test_node")

  # wait for the server to actualy show up before doing anything
  manipulatorClient.wait_for_server()

  joystickSubscriber = rospy.Subscriber('joy', Joy, JoystickCallback)

  rospy.spin()

if __name__ == "__main__":
  main()
