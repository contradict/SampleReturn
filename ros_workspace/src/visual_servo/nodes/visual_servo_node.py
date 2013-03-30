#!/usr/bin/env python
import sys
import rospy
import rosnode
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo

# TODO
# Remap the inputs and outputs in the launch file and create better names for the publish and subscription
# strings here.

class VisualServo:
	"""A class to position the robot so that it can pick up an object with the manipulator"""
	
	def __init__(self):
		rospy.init_node('visual_servo')
		rospy.Subscriber('/sample_detection/red_puck_imgpoints', Point, self.object_point_callback)
		rospy.Subscriber('/navigation/left/camera_info', CameraInfo, self.camera_left_info_callback)
		rospy.Subscriber('/navigation/right/camera_info', CameraInfo, self.camera_right_info_callback)
		self._publisher = rospy.Publisher('/odometry/twist', Twist)
		self._camera_left_width = -1
		self._camera_left_height = -1
		self._camera_right_width = -1
		self._camera_right_height = -1
		self._camera_left_width_window = 1
		self._camera_left_height_window = 1
		self._camera_right_width_window = 1
		self._camera_right_height_window = 1
		self._camera_left_target_height = 400
		self._camera_right_target_height = 400
		self._is_trying_to_work = True

	def object_point_callback(self, data):
		# This function does the bulk of the work.  It receives a centroid location of an object in pixel space
		# and tries to get the robot to move so that the centroid is correctly positioned in the image
		# such that the manipulator arm can pick up the object
		if self._camera_left_width > 0 and self._camera_left_height > 0 and self._camera_right_width > 0 and self._camera_right_height > 0:
			rospy.loginfo(rospy.get_name() + ": received point %s" % data)
			self._is_trying_to_work = True
			twist = Twist()

			# If the object is near the top of the image, move forward
			if data.y < self._camera_left_height*0.1:
				twist.angular.z = 0.0
				twist.angular.y = 0.0
				twist.angular.x = 0.0
				twist.linear.x = 0.0
				twist.linear.y = 0.02
				twist.linear.z = 0.0
			# Else if the object is near the bottom of the image, move backwards
			if data.y > self._camera_left_height*0.9:
				twist.angular.z = 0.0
				twist.angular.y = 0.0
				twist.angular.x = 0.0
				twist.linear.x = 0.0
				twist.linear.y = -0.02
				twist.linear.z = 0.0
			# Else if the object is not centered along the horizontal axis, rotate until it is
			elif data.x > (self._camera_left_width/2.0)+self._camera_left_width_window:
				twist.angular.z = 0.01
				twist.angular.y = 0.0
				twist.angular.x = 0.0
				twist.linear.x = 0.0
				twist.linear.y = 0.0
				twist.linear.z = 0.0
			elif data.x < (self._camera_left_width/2.0)-self._camera_left_width_window:
				twist.angular.z = -0.01
				twist.angular.y = 0.0
				twist.angular.x = 0.0
				twist.linear.x = 0.0
				twist.linear.y = 0.0
				twist.linear.z = 0.0
			# Else if the object is not aligned at the right location vertically in the image, move forward or backwards
			elif data.y < (self._camera_left_target_height-self._camera_left_height_window):
				twist.angular.z = 0.0
				twist.angular.y = 0.0
				twist.angular.x = 0.0
				twist.linear.x = 0.0
				twist.linear.y = 0.02
				twist.linear.z = 0.0
			elif data.y > (self._camera_left_target_height+self._camera_left_height_window):
				twist.angular.z = 0.0
				twist.angular.y = 0.0
				twist.angular.x = 0.0
				twist.linear.x = 0.0
				twist.linear.y = -0.02
				twist.linear.z = 0.0
			# Else the object should be aligned at the proper location in the image
			else:
				# TODO - Publish a message to let the executive know that the object is aligned
				twist.angular.z = 0.0
				twist.angular.y = 0.0
				twist.angular.x = 0.0
				twist.linear.x = 0.0
				twist.linear.y = 0.0
				twist.linear.z = 0.0

			self._publisher.publish(twist)

	def camera_left_info_callback(self, data):
		self._camera_left_width = data.width;
		self._camera_left_height = data.height;
		self._camera_left_width_window = max(1, data.width*0.01)
		self._camera_left_height_window = max(1, data.height*0.01)

	def camera_right_info_callback(self, data):
		self._camera_right_width = data.width;
		self._camera_right_height = data.height;
		self._camera_right_width_window = max(1, data.width*0.01)
		self._camera_right_height_window = max(1, data.height*0.01)

	def do_visual_servo(self):
		self._sleep_duration = rospy.Duration(10)
		while not rospy.is_shutdown():
			self._is_trying_to_work = False
			rospy.sleep(self._sleep_duration)
			if not self._is_trying_to_work:
				# TODO - Let the executive ROS node know that this node has failed to do anything useful for a while
				rospy.logerr("Failed to do any work for ten seconds!")

if __name__ == '__main__':
	visual_servo = VisualServo()
	visual_servo.do_visual_servo()
