#!/usr/bin/python
import math

import numpy as np

import roslib; roslib.load_manifest("platform_motion")
import rospy
import actionlib

from std_msgs.msg import Float64
from sensor_msgs.msg import Joy, JointState

from platform_motion.msg import SelectCarouselBinAction, SelectCarouselBinFeedback, SelectCarouselBinResult
from platform_motion.srv import Enable
from platform_motion.msg import HomeAction, HomeGoal

class CarouselIndexer(object):
    _selectBinFeedback = SelectCarouselBinFeedback()
    _selectBinResult = SelectCarouselBinResult()
    def __init__(self, bins):
        self.carousel_bin_angles = np.array(bins);
        self.current_index = None
        self.angle_pub = rospy.Publisher('carousel', Float64)
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joyCallback)
        self.joint_sub = rospy.Subscriber('platform_joint_state', JointState,
                self.jointStateCallback)

        self.goal_index = None
        self.server = actionlib.SimpleActionServer('select_carousel_bin',
                SelectCarouselBinAction, auto_start=False)
        self.server.register_goal_callback(self.selectBin)
        self.server.register_preempt_callback(self.preempt)
        self.server.start()
        print "Waiting for enable service"
        rospy.wait_for_service("enable_carousel")
        self.enable_service = rospy.ServiceProxy('enable_carousel', Enable)
        self.home_client = actionlib.SimpleActionClient('home_carousel',
                HomeAction)
        print "Waiting for home server"
        self.home_client.wait_for_server()
        print "ready"

    def joyCallback(self, joy_msg):
        if(self.server.is_active()):
            print "server active, joystick ignored"
            return
        if(self.current_index is not None):
            if(joy_msg.axes[4]<0):
                print "increment"
                index = self.current_index + 1
                index %= len(self.carousel_bin_angles)
                self.send_carousel_position(index)
            elif(joy_msg.axes[4]>0):
                print "decrement"
                index = self.current_index - 1
                index %= len(self.carousel_bin_angles)
                self.send_carousel_position(index)
        if(joy_msg.buttons[0]):
            print "enable"
            self.enable_service(True)
        elif(joy_msg.buttons[3]):
            print "disable"
            self.enable_service(False)
        elif(joy_msg.buttons[4]):
            print "home"
            self.home_client.send_goal(HomeGoal())

    def jointStateCallback(self, joint_msg):
        if self.current_index is None:
            print "got joint message"
        carousel_position =\
                joint_msg.position[joint_msg.name.index("carousel_joint")]
        self.current_index = np.argmin(
                np.abs(self.carousel_bin_angles -
                       carousel_position%(2*math.pi)))
        if self.server.is_active():
            if self.current_index == self.goal_index:
                self._selectBinResult.bin_selected = True
                self.server.set_succeeded(result=self._selectBinResult)
            else:
                self._selectBinFeedback.current_bin = self.current_index
                self.server.publish_feedback(self._selectBinFeedback)

    def selectBin(self):
        goal = self.server.accept_new_goal()
        if self.current_index is None:
            rospy.logerr("bin slection goal received, but no joint_state yet")
            self.server.set_aborted(text="bin slection goal received, but no joint_state yet")
            return
        if goal.bin_index<0 or goal.bin_index>=len(self.carousel_bin_angles):
            msg="bin slection goal received, index %d out of range [0, %d)"%(
                goal.bin_index, len(self.carousel_bin_angles))
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            return
        else:
            rospy.loginfo("accept goal for bin index %d", goal.bin_index)
            self.goal_index = goal.bin_index
            self.send_carousel_position(goal.bin_index)

    def preempt(self):
        self.send_carousel_position(self.current_index)
        self._selectBinResult.bin_selected = False
        self.server.set_preempted(result=self._selectBinResult)
 
    def send_carousel_position(self, index):
        angle = self.carousel_bin_angles[index]
        self.angle_pub.publish(Float64(angle))

if __name__ == "__main__":
    rospy.init_node("carousel_indexer")
    ci = CarouselIndexer(np.linspace(0,2*math.pi-2*math.pi/10,10))
    rospy.spin()

