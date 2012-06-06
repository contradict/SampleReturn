#!/usr/bin/python

import roslib; roslib.load_manifest("manipulator")
import rospy
import actionlib

from manipulator.msg import ManipulatorGrabAction, ManipulatorGrabGoal, ManipulatorGrabFeedback, ManipulatorGrabResult

from sensor_msgs.msg import Joy

def JoystickCallback(msg):
  # when the right button is pushed, emit a manipulator grab command!
  if msg.buttons[self.buttonGrab] == 1:
    goal = ManipulatorGrabGoal()
    goal.wrist_angle = 0.0
    self.manipulatorClient.send(goal)
    self.manipulatorClient.wait_for_result()

def main():
  rospy.init_node("manipulator_test_node")

  self.buttonGrab = rospy.param('button_grab', 4)

  self.manipulatorClient = actionlib.SimpleActionClient(
      'manipulator_grab',
      ManipulatorGrabAction
  )

  # wait for the server to actualy show up before doing anything
  self.manipulatorClient.wait_for_server()

  joystickSubscriber = rospy.Subscriber('joy', Joy, JoystickCallback)

  rospy.spin()

if __name__ == "__main__":
  main()
