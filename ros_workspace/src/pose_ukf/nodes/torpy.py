#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Vector3
from sensor_msgs.msg import Imu
import tf

class ToRPY(object):
    def __init__(self):
        self.pub = rospy.Publisher('~rpy', Vector3, queue_size=1)
        self.sub = rospy.Subscriber('pose', PoseStamped, self.resendPose)
        self.sub = rospy.Subscriber('imu', Imu, self.resendImu)

    def sendrpy(self, q):
        v = Vector3(*tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w]))
        self.pub.publish(v)

    def resendPose(self, msg):
        self.sendrpy(msg.pose.orientation)

    def resendImu(self, msg):
        self.sendrpy(msg.orientation)

if __name__ == '__main__':
    rospy.init_node("torpy")
    to = ToRPY()
    rospy.spin()
