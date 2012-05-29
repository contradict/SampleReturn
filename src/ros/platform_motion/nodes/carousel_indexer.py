#!/usr/bin/python
import math

import roslib; roslib.load_manifest("platform_motion")
import rospy
import actionlib

from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Joy, JointState

from platform_motion.msg import SelectCarouselBinAction, SelectCarouselBinFeedback, SelectCarouselBinResult

class CarouselIndexer(object):
    _selectBinFeedback = SelectCarouselBinFeedback()
    _selectBinResult = SelectCarouselBinResult()
    def __init__(self, bins):
        self.carousel_index_count = bins;
        self.current_index = None
        self.quat_pub = rospy.Publisher('carousel', Quaternion)
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joyCallback)
        self.joint_sub = rospy.Subscriber('platform_joint_state', JointState,
                self.jointStateCallback)

        self.goal_index = None
        self.server = actionlib.SimpleActionServer('select_carousel_bin', SelectCarouselBinAction)
        self.server.register_goal_callback(self.selectBin)
        self.server.register_preempt_callback(self.preempt)
        self.server.start()

    def joyCallback(self, joy_msg):
        if(joy_msg.axes[4] == 0 or
                self.current_index is None or
                self.server.is_active()):
            return
        elif(joy_msg.axes[4]<0):
            index = self.current_index + 1
        elif(joy_msg.axes[4]>0):
            index = self.current_index - 1
        index %= self.carousel_index_count
        self.send_carousel_position(index)

    def jointStateCallback(self, joint_msg):
        carousel_position =\
                joint_msg.position[joint_msg.name.index("carousel_joint")]
        self.current_index = \
                carousel_position/2./math.pi*self.carousel_index_count;
        self.current_index %= self.carousel_index_count
        if self.server.is_active():
            if self.current_index == self.goal_index:
                _selectBinResult.bin_selected = True
                self.server.set_succeeded(result=_selectBinResult)
            else:
                _selectBinFeedback.current_bin = self.current_index
                self.server.publish_feedback(_selectBinFeedback)

    def selectBin(self, goal):
        if self.current_index is None:
            rospy.logerr("bin slection goal received, but no joint_state yet")
            self.server.setRejected()
        elif goal.bin_index<0 or goal.bin_index>=self.carousel_index_count:
            rospy.logerr("bin slection goal received, index %d out of range [0, %d)",
                goal.bin_index, self.carousel_index_count)
            self.server.setRejected()
        else:
            rospy.loginfo("accept goal for bin index %d", goal.bin_index)
            self.goal_index = goal.bin_index
            self.server.accept_new_goal()
            self.send_carousel_position(goal.bin_index)

    def preempt(self):
        self.send_carousel_position(self.current_index)
        _selectBinResult.bin_selected = False
        self.server.set_preempted(result=_selectBinResult)
 
    def send_carousel_position(self, index):
        angle = index*2*math.pi/self.carousel_index_count
        Q = Quaternion(0,0,sin(angle/2.), cos(angle/2.))
        self.quat_pub.publish(Q)

if __name__ == "__main__":
    rospy.init_node("carousel_indexer")
    ci = CarouselIndexer(10)
    rospy.spin()

