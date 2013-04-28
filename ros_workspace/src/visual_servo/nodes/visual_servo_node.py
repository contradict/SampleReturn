#!/usr/bin/env python
import sys
import rospy
import rosnode
import actionlib
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo
from visual_servo.msg import VisualServoAction, VisualServoResult, VisualServoFeedback

# TODO
# Remap the inputs and outputs in the launch file and create better names for the publish and subscription
# strings here.

def enum(**enums):
	return type('Enum', (), enums)

VisualServoStates = enum(SAFE_REGION=0, ROTATE=1, MOVE_FORWARD=2)

class VisualServo:
	"""A class to position the robot so that it can pick up an object with the manipulator"""
	
	def __init__(self):
		rospy.Subscriber('/red_puck_imgpoints', PointStamped, self.object_point_callback)
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
		self._camera_left_target_width = 317
		self._camera_right_target_width = 150
		self._camera_left_target_height = 256
		self._camera_right_target_height = 256
		self._camera_left_target_forward_vector_x = -16
		self._camera_left_target_forward_vector_y = -255
		self._camera_right_target_forward_vector_x = 1
		self._camera_right_target_forward_vector_y = -1
		self._proportional_constant_drive_forward = 0.01
		self._integral_constant_drive_forward = 0.0
		self._derivative_constant_drive_forward = 0.0
		self._proportional_constant_rotate = 0.01
		self._integral_constant_rotate = 0.0
		self._derivative_constant_rotate = 0.0
		self._visual_servo_state = VisualServoStates.SAFE_REGION
		self._is_trying_to_work = True
		self._has_succeeded = False
		self._previous_time = False
		self.reset_pid_controller()
		self._visual_servo_result = VisualServoResult()
		self._visual_servo_feedback = VisualServoFeedback()
		self._action_server = actionlib.SimpleActionServer('visual_servo_action', VisualServoAction, self.do_visual_servo, False)
		self._action_server.start()

	def get_target_forward_line_width_at_height(self, height):
		return (height - self._camera_left_target_height)*self._camera_left_target_forward_vector_x/self._camera_left_target_forward_vector_y + self._camera_left_target_width

	def reset_pid_controller(self):
		self._previous_error = 0
		self._integral_error = 0

	def update_pid_controller(self, error, delta_time):
		if delta_time < 0.000001:
			pid_controller_output = 0
		else:
			if self._visual_servo_state == VisualServoStates.SAFE_REGION or self._visual_servo_state == VisualServoStates.MOVE_FORWARD:
				k_p = self._proportional_constant_drive_forward
				k_i = self._integral_constant_drive_forward
				k_d = self._derivative_constant_drive_forward
			elif self._visual_servo_state == VisualServoStates.ROTATE:
				k_p = self._proportional_constant_rotate
				k_i = self._integral_constant_rotate
				k_d = self._derivative_constant_rotate
			self._integral_error = self._integral_error + error*delta_time
			derivative = (error - self._previous_error)/delta_time
			pid_controller_output = k_p*error + k_i*self._integral_error + k_d*derivative
		self._previous_error = error
		return pid_controller_output

	def object_point_callback(self, data):
		# This function does the bulk of the work.  It receives a centroid location of an object in pixel space
		# and tries to get the robot to move so that the centroid is correctly positioned in the image
		# such that the manipulator arm can pick up the object
		if self._camera_left_width > 0 and self._camera_left_height > 0 and self._camera_right_width > 0 and self._camera_right_height > 0:
			rospy.loginfo(rospy.get_name() + ": received point %s" % data.point)
			self._is_trying_to_work = True

			if not self._previous_time:
				self._previous_time = data.header.stamp

			twist = Twist()
			twist.angular.z = 0.0
			twist.angular.y = 0.0
			twist.angular.x = 0.0
			twist.linear.x = 0.0
			twist.linear.y = 0.0
			twist.linear.z = 0.0

			# Set the action server error feedback to the distance between the centroid and the target location
			self._visual_servo_feedback.error = sqrt((data.point.x-self._camera_left_target_width)*(data.point.x-self._camera_left_target_width)+(data.point.y-self._camera_left_target_height)*(data.point.y-self._camera_left_target_height))

			if self._visual_servo_state == VisualServoStates.SAFE_REGION:
				# If the object is near the top of the image, move forward
				if data.point.y < self._camera_left_height*0.1:
					error = self._camera_left_height*0.15 - data.point.y
					delta_time = 0.1
					pid_controller_output = self.update_pid_controller(error, delta_time)
					twist.angular.z = 0.0
					twist.angular.y = 0.0
					twist.angular.x = 0.0
					twist.linear.x = pid_controller_output
					twist.linear.y = 0.0
					twist.linear.z = 0.0
				# Else if the object is near the bottom of the image, move backwards
				elif data.point.y > self._camera_left_height*0.9:
					error = data.point.y - self._camera_left_height*0.85
					delta_time = (data.header.stamp-self._previous_time).to_sec()
					pid_controller_output = self.update_pid_controller(error, delta_time)
					twist.angular.z = 0.0
					twist.angular.y = 0.0
					twist.angular.x = 0.0
					twist.linear.x = -pid_controller_output
					twist.linear.y = 0.0
					twist.linear.z = 0.0
				else:
					self._visual_servo_state = VisualServoStates.ROTATE
					self.reset_pid_controller()

			if self._visual_servo_state == VisualServoStates.ROTATE:
				target_width = self.get_target_forward_line_width_at_height(data.point.y)
				# If the object is not at the target width, rotate until it is
				if data.point.x > target_width+self._camera_left_width_window:
					error = target_width - data.point.x
					delta_time = (data.header.stamp-self._previous_time).to_sec()
					pid_controller_output = self.update_pid_controller(error, delta_time)
					twist.angular.z = -pid_controller_output
					twist.angular.y = 0.0
					twist.angular.x = 0.0
					twist.linear.x = 0.0
					twist.linear.y = 0.0
					twist.linear.z = 0.0
				elif data.point.x < target_width-self._camera_left_width_window:
					error = target_width - data.point.x
					delta_time = (data.header.stamp-self._previous_time).to_sec()
					pid_controller_output = self.update_pid_controller(error, delta_time)
					twist.angular.z = pid_controller_output
					twist.angular.y = 0.0
					twist.angular.x = 0.0
					twist.linear.x = 0.0
					twist.linear.y = 0.0
					twist.linear.z = 0.0
				else:
					self._visual_servo_state = VisualServoStates.MOVE_FORWARD
					self.reset_pid_controller()

			if self._visual_servo_state == VisualServoStates.MOVE_FORWARD:
				# Make sure the object is still aligned with the forward line
				target_width = self.get_target_forward_line_width_at_height(data.point.y)
				if data.point.x > target_width+self._camera_left_width_window or data.point.x < target_width-self._camera_left_width_window:
					self._visual_servo_state = VisualServoStates.ROTATE
					self.reset_pid_controller()
				else:
					# If the object is not aligned at the right location along the forward vector in the image, move forward or backwards
					if data.point.y < (self._camera_left_target_height-self._camera_left_height_window):
						error = self._camera_left_target_height-self._camera_left_height_window-data.point.y
						delta_time = (data.header.stamp-self._previous_time).to_sec()
						pid_controller_output = self.update_pid_controller(error, delta_time)
						twist.angular.z = 0.0
						twist.angular.y = 0.0
						twist.angular.x = 0.0
						twist.linear.x = pid_controller_output
						twist.linear.y = 0.0
						twist.linear.z = 0.0
					elif data.point.y > (self._camera_left_target_height+self._camera_left_height_window):
						error = data.point.y-(self._camera_left_target_height+self._camera_left_height_window)
						delta_time = (data.header.stamp-self._previous_time).to_sec()
						pid_controller_output = self.update_pid_controller(error, delta_time)
						twist.angular.z = 0.0
						twist.angular.y = 0.0
						twist.angular.x = 0.0
						twist.linear.x = -pid_controller_output
						twist.linear.y = 0.0
						twist.linear.z = 0.0
					# Else the object should be aligned at the proper location in the image
					else:
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

	def do_visual_servo(self, goal):
		self._sleep_duration = rospy.Duration(2)
		while not rospy.is_shutdown():
			self._is_trying_to_work = False
			rospy.sleep(self._sleep_duration)
			if self._has_succeeded:
				self._visual_servo_result.success = True
				self._action_server.set_succeeded(result=self._visual_servo_result)
				break
			if not self._is_trying_to_work:
				self._visual_servo_result.success = False
				self._action_server.set_succeeded(result=self._visual_servo_result)
				rospy.logerr("Failed to do any work for two seconds!")
			else:
				self._action_server.publish_feedback(self._visual_servo_feedback)

if __name__ == '__main__':
	rospy.init_node('visual_servo')
	visual_servo = VisualServo()
	#visual_servo.do_visual_servo(True)
	rospy.spin()
