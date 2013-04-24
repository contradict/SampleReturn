#!/usr/bin/env python
import sys
import rospy
import rosnode
import actionlib
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo
from visual_servo.msg import VisualServoAction, VisualServoResult, VisualServoFeedback

# TODO
# Remap the inputs and outputs in the launch file and create better names for the publish and subscription
# strings here.

class VisualServo:
	"""A class to position the robot so that it can pick up an object with the manipulator"""
	
	def __init__(self):
		rospy.Subscriber('/red_puck_imgpoints', Point, self.object_point_callback)
		rospy.Subscriber('/navigation/left/camera_info', CameraInfo, self.camera_left_info_callback)
		rospy.Subscriber('/navigation/right/camera_info', CameraInfo, self.camera_right_info_callback)
		self._publisher = rospy.Publisher('/servo_command', Twist)
		self._camera_left_width = -1
		self._camera_left_height = -1
		self._camera_right_width = -1
		self._camera_right_height = -1
		self._camera_left_width_window = 3
		self._camera_left_height_window = 5
		self._camera_right_width_window = 3
		self._camera_right_height_window = 5
		self._camera_left_target_width = 330
		self._camera_right_target_width = 150
		self._camera_left_target_height = 256
		self._camera_right_target_height = 256
		self._rotation_velocity = 0.1
		self._linear_velocity = 0.1
		self._is_trying_to_work = True
		self._has_succeeded = False
		self._visual_servo_result = VisualServoResult()
		self._visual_servo_feedback = VisualServoFeedback()
		self._action_server = actionlib.SimeActionServer('visual_servo_action', VisualServoAction, self.do_visual_servo, False)
		self._action_server.start()

	def object_point_callback(self, data):
		# This function does the bulk of the work.  It receives a centroid location of an object in pixel space
		# and tries to get the robot to move so that the centroid is correctly positioned in the image
		# such that the manipulator arm can pick up the object
		if self._camera_left_width > 0 and self._camera_left_height > 0 and self._camera_right_width > 0 and self._camera_right_height > 0:
			rospy.loginfo(rospy.get_name() + ": received point %s" % data)
			self._is_trying_to_work = True
			twist = Twist()

			# TODO - Set this error appropriately
			self._visual_servo_feedback.error = sqrt((data.x-self._camera_left_target_width)*(data.x-self._camera_left_target_width)+(data.y-self._camera_left_target_height)*(data.y-self._camera_left_target_height))

			# If the object is near the top of the image, move forward
			if data.y < self._camera_left_height*0.1:
				twist.angular.z = 0.0
				twist.angular.y = 0.0
				twist.angular.x = 0.0
				twist.linear.x = self._linear_velocity
				twist.linear.y = 0.0
				twist.linear.z = 0.0
			# Else if the object is near the bottom of the image, move backwards
			if data.y > self._camera_left_height*0.9:
				twist.angular.z = 0.0
				twist.angular.y = 0.0
				twist.angular.x = 0.0
				twist.linear.x = -self._linear_velocity
				twist.linear.y = 0.0
				twist.linear.z = 0.0
			# Else if the object is not at the target width, rotate until it is
			elif data.x > self._camera_left_target_width+self._camera_left_width_window:
				twist.angular.z = -self._rotation_velocity
				twist.angular.y = 0.0
				twist.angular.x = 0.0
				twist.linear.x = 0.0
				twist.linear.y = 0.0
				twist.linear.z = 0.0
			elif data.x < self._camera_left_target_width-self._camera_left_width_window:
				twist.angular.z = self._rotation_velocity
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
				twist.linear.x = self._linear_velocity
				twist.linear.y = 0.0
				twist.linear.z = 0.0
			elif data.y > (self._camera_left_target_height+self._camera_left_height_window):
				twist.angular.z = 0.0
				twist.angular.y = 0.0
				twist.angular.x = 0.0
				twist.linear.x = -self._linear_velocity
				twist.linear.y = 0.0
				twist.linear.z = 0.0
			# Else the object should be aligned at the proper location in the image
			else:
				self._visual_servo_result.success = True
				self._has_succeeded = True
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
		self._sleep_duration = rospy.Duration(2)
		while not rospy.is_shutdown():
			self._is_trying_to_work = False
			rospy.sleep(self._sleep_duration)
			if self._has_succeeded:
				self._action_server.set_succeeded(result=self._visual_servo_result)
				break
			if not self._is_trying_to_work:
				# TODO - Let the executive ROS node know that this node has failed to do anything useful for a while
				twist = Twist()
				twist.angular.z = 0.0
				twist.angular.y = 0.0
				twist.angular.x = 0.0
				twist.linear.x = 0.0
				twist.linear.y = 0.0
				twist.linear.z = 0.0
				self._publisher.publish(twist)
				rospy.logerr("Failed to do any work for two seconds!")
			else:
				self._action_server.publish_feedback(self._visual_servo_feedback)

if __name__ == '__main__':
	rospy.init_node('visual_servo')
	visual_servo = VisualServo()
	visual_servo.do_visual_servo()
