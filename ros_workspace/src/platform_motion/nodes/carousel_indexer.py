#!/usr/bin/python
import math

import numpy as np

import roslib; roslib.load_manifest("platform_motion")
import rospy
import actionlib

from std_msgs.msg import Float64
from sensor_msgs.msg import Joy, JointState

from platform_motion_msgs.msg import SelectCarouselBinAction, SelectCarouselBinFeedback, SelectCarouselBinResult
from platform_motion_msgs.srv import Enable
from platform_motion_msgs.msg import HomeAction, HomeGoal

class CarouselIndexer(object):
    _selectBinFeedback = SelectCarouselBinFeedback()
    _selectBinResult = SelectCarouselBinResult()
    def __init__(self, bins, bin_tolerance):
        self.carousel_bin_angles = np.array(bins);
        self.bin_tolerance = bin_tolerance
        self.carousel_position = None
        self.current_index = None
        self.joy_goal = None
        self.angle_pub = rospy.Publisher('carousel_angle', Float64)
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joyCallback)
        self.joint_sub = rospy.Subscriber('platform_joint_state', JointState,
                self.jointStateCallback)
        self.goal_index = None
        self.server = actionlib.SimpleActionServer('select_carousel_bin',
                SelectCarouselBinAction, auto_start=False)
        self.server.register_goal_callback(self.selectBin)
        self.server.register_preempt_callback(self.preempt)
        self.server.start()
        rospy.loginfo( "Waiting for carousel enable service" )
        rospy.wait_for_service("enable_carousel")
        self.enable_service = rospy.ServiceProxy('enable_carousel', Enable)
        self.home_client = actionlib.SimpleActionClient('home_carousel',
                HomeAction)
        rospy.loginfo( "Waiting for home server" )
        self.home_client.wait_for_server()
        rospy.loginfo( "ready" )

    def joyCallback(self, joy_msg):
        if(joy_msg.axes[4] != 0):
            if self.server.is_active():
                rospy.logwarn( "server active, joystick ignored" )
                return
            if self.joy_goal is None:
               self.joy_goal = self.current_index
            if(joy_msg.axes[4]<0):
                rospy.loginfo( "increment" )
                index = self.joy_goal + 1
                index %= len(self.carousel_bin_angles)
                self.joy_goal = index
                self.send_carousel_position(index)
            elif(joy_msg.axes[4]>0):
                rospy.loginfo( "decrement" )
                index = self.joy_goal - 1
                index %= len(self.carousel_bin_angles)
                self.joy_goal = index
                self.send_carousel_position(index)
        if(joy_msg.axes[5] > 0):
            rospy.loginfo( "enable" )
            try:
                self.enable_service(True)
                self.joy_goal = self.current_index
            except rospy.ServiceException, e:
                rospy.logerr("Unable to enable carousel: %s", e)
        elif(joy_msg.axes[5] < 0):
            rospy.loginfo( "disable" )
            try:
                self.enable_service(False)
            except rospy.ServiceException, e:
                rospy.logerr("Unable to disable carousel: %s", e)
        elif(joy_msg.buttons[8]):
            rospy.loginfo( "home" )
            self.home_client.send_goal(HomeGoal())
            self.joy_goal = 0

    def jointStateCallback(self, joint_msg):
        if self.carousel_position is None:
            rospy.loginfo( "got joint message" )
        self.carousel_position =\
                joint_msg.position[joint_msg.name.index("carousel_joint")]
        self.despun_position = self.carousel_position%(2*math.pi)
        if self.despun_position>math.pi:
            self.despun_position -= 2*math.pi
        bin_distance = np.abs( self.carousel_bin_angles -
                               self.despun_position )
        closest_bin = np.argmin(bin_distance)
        if bin_distance[closest_bin] < self.bin_tolerance:
            if closest_bin != self.current_index:
                rospy.loginfo( "at bin %d", closest_bin)
                self.current_index = closest_bin
        if self.server.is_active():
            self._selectBinFeedback.current_bin = self.current_index
            self.server.publish_feedback(self._selectBinFeedback)
            if self.current_index == self.goal_index:
                self._selectBinResult.bin_selected = True
                self.server.set_succeeded(result=self._selectBinResult)

    def selectBin(self):
        goal = self.server.accept_new_goal()
        if self.current_index is None:
            rospy.logerr("bin selection goal received, but no joint_state yet")
            self.server.set_aborted(text="bin selection goal received, but no joint_state yet")
            return
        if goal.bin_index<0 or goal.bin_index>=len(self.carousel_bin_angles):
            msg="bin selection goal received, index %d out of range [0, %d)"%(
                goal.bin_index, len(self.carousel_bin_angles))
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            return
        else:
            rospy.loginfo("accept goal for bin index %d", goal.bin_index)
            self.goal_index = goal.bin_index
            self.joy_goal = self.goal_index
            self.send_carousel_position(goal.bin_index)

    def preempt(self):
        rospy.loginfo("Preempting carousel indexer")
        self.send_carousel_position(self.current_index)
        self._selectBinResult.bin_selected = False
        self.server.set_preempted(result=self._selectBinResult)
 
    def send_carousel_position(self, index):
        if self.carousel_position is None:
            rospy.logerr( "No position yet" )
        requested_angle = self.carousel_bin_angles[index]
        delta_angle = self.despun_position - requested_angle
        if delta_angle>math.pi:
            delta_angle -= 2*math.pi
        if delta_angle<-math.pi:
            delta_angle += 2*math.pi
        rospy.loginfo("r: %f despun: %f delta: %f", requested_angle, self.despun_position, delta_angle)
        angle = self.carousel_position - delta_angle
        rospy.loginfo( "sending %d: %f", index, angle )
        self.angle_pub.publish(Float64(angle))
   

if __name__ == "__main__":
    rospy.init_node("carousel_indexer")
    pos_bin_angles = np.radians( [0, 48.5, 73.5, 98.0, 127.5, 162.5])
    bin_angles = np.r_[pos_bin_angles, -pos_bin_angles[:0:-1]]
    rospy.loginfo("bin angles: %s", bin_angles)
    ci = CarouselIndexer(bin_angles, 0.02)
    rospy.spin()

