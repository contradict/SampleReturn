#!/usr/bin/env python
import sys
import rospy
import rosnode
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped

class VisualServo:
	"""A class to position the robot so that it can pick up an object with the manipulator"""
	
	def __init__(self):
		rospy.init_node('visual_servo')
		rospy.Subscriber('/object_point', PointStamped, self.object_point_callback)
		self._publisher = rospy.Publisher('/odometry/twist', Twist)

	def object_point_callback(self, data):
		rospy.loginfo(rospy.get_name() + ": received point %s" % data.data)
		twist = Twist()
		twist.angular.z = 0.1
		twist.angular.y = 0.0
		twist.angular.x = 0.0
		twist.linear.x = 0.0
		twist.linear.y = 0.0
		twist.linear.z = 0.0
		self._publisher.publish(twist)

	def do_visual_servo(self):
		while not rospy.is_shutdown():
			rospy.spin()

if __name__ == '__main__':
	visual_servo = VisualServo()
	visual_servo.do_visual_servo()
