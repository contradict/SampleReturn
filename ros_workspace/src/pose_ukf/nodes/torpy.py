#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Vector3
from sensor_msgs.msg import Imu
import tf
import numpy as np

class ToRPY(object):
    def __init__(self):
        self.pub = rospy.Publisher('~rpy', Vector3, queue_size=1)
        self.pub_deg = rospy.Publisher('~rpy_deg', Vector3, queue_size=1)
        self.sub_pose = rospy.Subscriber('pose', PoseStamped, self.resendPose)
        self.sub_imu = rospy.Subscriber('imu', Imu, self.resendImu)

    def sendrpy(self, q):
        v = Vector3(*tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w]))
        self.pub.publish(v)
        v_deg = Vector3(np.degrees(v.x), np.degrees(v.y), np.degrees(v.z))
        self.pub_deg.publish(v_deg)

    def resendPose(self, msg):
        self.sendrpy(msg.pose.orientation)

    def resendImu(self, msg):
        self.sendrpy(msg.orientation)

if __name__ == '__main__':
    rospy.init_node("torpy")
    to = ToRPY()
    rospy.spin()
