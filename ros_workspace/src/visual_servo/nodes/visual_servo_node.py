#!/usr/bin/env python
import sys
import math
import numpy
import rospy
import rosnode
import actionlib
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo
from visual_servo.msg import VisualServoAction, VisualServoResult, VisualServoFeedback, TunableConstants

# TODO
# Remap the inputs and outputs in the launch file and create better names for the publish and subscription
# strings here.

def enum(**enums):
	return type('Enum', (), enums)

VisualServoStates = enum(SAFE_REGION=0, ROTATE=1, MOVE_FORWARD=2, STOP_AND_WAIT=3)

class VisualServo:
	"""A class to position the robot so that it can pick up an object with the manipulator"""
	
	def __init__(self):
		self._safe_region_percentage = rospy.get_param('~safe_region_percentage', 0.1)
		self._camera_left_image_width = -1
		self._camera_left_image_height = -1
		self._camera_right_image_width = -1
		self._camera_right_image_height = -1
		self._target_window_halfwidth_rotate = rospy.get_param('~target_window_halfwidth_rotate', 3)
		self._target_window_halfwidth_move_forward = rospy.get_param('~target_window_halfwidth_move_forward', 6)
		self._target_window_halfheight_move_forward = rospy.get_param('~target_window_halfheight_move_forward', 8)
		self._camera_left_target_pixel_x = rospy.get_param('~camera_left_target_pixel_x', 317)
		self._camera_right_target_pixel_x = rospy.get_param('~camera_right_target_pixel_x', 150)
		self._camera_left_target_pixel_y = rospy.get_param('~camera_left_target_pixel_y', 256)
		self._camera_right_target_pixel_y = rospy.get_param('~camera_right_target_pixel_y', 256)
		self._camera_left_target_forward_vector_x = rospy.get_param('~camera_left_target_forward_vector_x', -16)
		self._camera_left_target_forward_vector_y = rospy.get_param('~camera_left_target_forward_vector_y', -255)
		self._camera_right_target_forward_vector_x = rospy.get_param('~camera_right_target_forward_vector_x', 16)
		self._camera_right_target_forward_vector_y = rospy.get_param('~camera_right_target_forward_vector_y', 255)
		self._loop_frequency = rospy.get_param('~loop_frequency', 10.0)
		self._new_state_wait_time = rospy.get_param('~new_state_wait_time', 1.0)
		self._new_state_wait_timer = 0.0
		self._no_input_stop_wait_time = rospy.get_param('~no_input_stop_wait_time', 0.5)
		self._stop_wait_timer = 0.0
		self._proportional_constant_drive_forward = rospy.get_param('~proportional_constant_drive_forward', 0.01)
		self._integral_constant_drive_forward = rospy.get_param('~integral_constant_drive_forward', 0.0)
		self._derivative_constant_drive_forward = rospy.get_param('~derivative_constant_drive_forward', 0.0)
		self._proportional_constant_rotate = rospy.get_param('~proportional_constant_rotate', 0.01)
		self._integral_constant_rotate = rospy.get_param('~integral_constant_rotate', 0.0)
		self._derivative_constant_rotate = rospy.get_param('~derivative_constant_rotate', 0.0)
		self.goto_state(VisualServoStates.SAFE_REGION)
		self.reset()

		rospy.Subscriber('/red_puck_imgpoints', PointStamped, self.object_point_callback, None, 1)
		rospy.Subscriber('/navigation/left/camera_info', CameraInfo, self.camera_left_info_callback, None, 1)
		rospy.Subscriber('/navigation/right/camera_info', CameraInfo, self.camera_right_info_callback, None, 1)
		rospy.Subscriber('/debug/debug_update_constants', TunableConstants, self.debug_update_constants, None, 1)
		self._publisher = rospy.Publisher('/servo_command', Twist)

		self._visual_servo_result = VisualServoResult()
		self._visual_servo_feedback = VisualServoFeedback()
		self._action_server = actionlib.SimpleActionServer('visual_servo_action', VisualServoAction, self.run_visual_servo_action, False)
		self._action_server.start()

	def reset(self):
		self._has_succeeded = False
		self._previous_time = False
		self._previous_point = False
		self.reset_pid_controller()

	def debug_update_constants(self, tunable_constants):
		self._proportional_constant_drive_forward = tunable_constants.proportional_linear_constant
		self._integral_constant_drive_forward = tunable_constants.integral_linear_constant
		self._derivative_constant_drive_forward = tunable_constants.derivative_linear_constant
		self._proportional_constant_rotate = tunable_constants.proportional_angular_constant
		self._integral_constant_rotate = tunable_constants.integral_angular_constant
		self._derivative_constant_rotate = tunable_constants.derivative_angular_constant
		self._target_window_halfwidth_rotate = tunable_constants.target_window_halfwidth_rotate
		self._target_window_halfwidth_move_forward = tunable_constants.target_window_halfwidth_move_forward
		self._target_window_halfheight_move_forward = tunable_constants.target_window_halfheight_move_forward
		self._camera_left_target_pixel_x = tunable_constants.camera_left_target_pixel_x
		self._camera_left_target_pixel_y = tunable_constants.camera_left_target_pixel_y
		self._camera_right_target_pixel_x = tunable_constants.camera_right_target_pixel_x
		self._camera_right_target_pixel_y = tunable_constants.camera_right_target_pixel_y


	def goto_state(self, new_state):
		self._visual_servo_state = new_state
		self._new_state_wait_timer = self._new_state_wait_time
		self.reset_pid_controller()

	def get_target_forward_line_pixel_x(self, pixel_y):
		return (pixel_y - self._camera_left_target_pixel_y)*self._camera_left_target_forward_vector_x/self._camera_left_target_forward_vector_y + self._camera_left_target_pixel_x

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
		# This function receives a centroid location of an object in pixel space
		if self._camera_left_image_width > 0 and self._camera_left_image_height > 0 and self._camera_right_image_width > 0 and self._camera_right_image_height > 0:
			rospy.loginfo(rospy.get_name() + ": received point %s" % data.point)

			self._previous_point = data.point
			self._previous_time = data.header.stamp
			self._stop_wait_timer = 0.0

		# If we were stopped and waiting for input, restart!
		if self._visual_servo_state == VisualServoStates.STOP_AND_WAIT:
			self.goto_state(VisualServoStates.SAFE_REGION)

	def do_visual_servo(self):
		# This function does the visual servo work itself using the
		# centroid point received from object_point_callback().  It tries
		# to position the object such that the manipulator arm can pick it up

		# Early out if no object centroid point has been received yet
		if not self._previous_point or not self._previous_time:
			return

		point = self._previous_point
		delta_time = 1.0/self._loop_frequency
		twist = Twist()
		twist.angular.z = 0.0
		twist.angular.y = 0.0
		twist.angular.x = 0.0
		twist.linear.x = 0.0
		twist.linear.y = 0.0
		twist.linear.z = 0.0

		if self._new_state_wait_timer > 0:
			self.reset_pid_controller()
			self._new_state_wait_timer = self._new_state_wait_timer - delta_time

		self._stop_wait_timer = self._stop_wait_timer + delta_time
		if self._stop_wait_timer > self._no_input_stop_wait_time:
			self.goto_state(VisualServoStates.STOP_AND_WAIT)
			self._stop_wait_timer = 0.0

		# Set the action server error feedback to the distance between the centroid and the target location
		self._visual_servo_feedback.error = math.sqrt((point.x-self._camera_left_target_pixel_x)*(point.x-self._camera_left_target_pixel_x)+(point.y-self._camera_left_target_pixel_y)*(point.y-self._camera_left_target_pixel_y))

		# Debug output
		rospy.loginfo(rospy.get_name() + "Current state: %s" % self._visual_servo_state)

		if self._visual_servo_state == VisualServoStates.SAFE_REGION:
			# If the object is near the top of the image, move forward
			if point.y < self._camera_left_image_height*self._safe_region_percentage:
				error = self._camera_left_image_height*(self._safe_region_percentage+0.05) - point.y
				pid_controller_output = self.update_pid_controller(error, delta_time)
				twist.angular.z = 0.0
				twist.angular.y = 0.0
				twist.angular.x = 0.0
				twist.linear.x = pid_controller_output
				twist.linear.y = 0.0
				twist.linear.z = 0.0
			# Else if the object is below the target point, move backwards
			elif point.y > self._camera_left_target_pixel_y+(self._camera_left_image_height*self._safe_region_percentage):
				error = point.y - self._camera_left_target_pixel_y+(self._camera_left_image_height*self._safe_region_percentage)
				pid_controller_output = self.update_pid_controller(error, delta_time)
				twist.angular.z = 0.0
				twist.angular.y = 0.0
				twist.angular.x = 0.0
				twist.linear.x = -pid_controller_output
				twist.linear.y = 0.0
				twist.linear.z = 0.0
			else:
				self.goto_state(VisualServoStates.ROTATE)

		if self._visual_servo_state == VisualServoStates.ROTATE:
			target_width = self.get_target_forward_line_pixel_x(point.y)
			# If the object is not at the target width, rotate until it is
			if point.x > target_width+self._target_window_halfwidth_rotate:
				error = point.x - target_width
				pid_controller_output = self.update_pid_controller(error, delta_time)
				twist.angular.z = -pid_controller_output
				twist.angular.y = 0.0
				twist.angular.x = 0.0
				twist.linear.x = 0.0
				twist.linear.y = 0.0
				twist.linear.z = 0.0
			elif point.x < target_width-self._target_window_halfwidth_rotate:
				error = target_width - point.x
				pid_controller_output = self.update_pid_controller(error, delta_time)
				twist.angular.z = pid_controller_output
				twist.angular.y = 0.0
				twist.angular.x = 0.0
				twist.linear.x = 0.0
				twist.linear.y = 0.0
				twist.linear.z = 0.0
			else:
				self.goto_state(VisualServoStates.MOVE_FORWARD)

		if self._visual_servo_state == VisualServoStates.MOVE_FORWARD:
			# Make sure the object is still aligned with the forward line
			target_width = self.get_target_forward_line_pixel_x(point.y)
			if point.x > target_width+self._target_window_halfwidth_move_forward or point.x < target_width-self._target_window_halfwidth_move_forward:
				self.goto_state(VisualServoStates.ROTATE)
			else:
				# If the object is not aligned at the right location along the forward vector in the image, move forward or backwards
				if point.y < (self._camera_left_target_pixel_y-self._target_window_halfheight_move_forward):
					error = self._camera_left_target_pixel_y-self._target_window_halfheight_move_forward-point.y
					pid_controller_output = self.update_pid_controller(error, delta_time)
					twist.angular.z = 0.0
					twist.angular.y = 0.0
					twist.angular.x = 0.0
					twist.linear.x = pid_controller_output
					twist.linear.y = 0.0
					twist.linear.z = 0.0
				elif point.y > (self._camera_left_target_pixel_y+self._target_window_halfheight_move_forward):
					error = point.y-(self._camera_left_target_pixel_y+self._target_window_halfheight_move_forward)
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

		if self._visual_servo_state == VisualServoStates.STOP_AND_WAIT:
			# This handles cases where no input is received from the system
			# for a while.  In this case, slow the robot down to a stop and
			# wait for the input to start up again.  This employs a Hermitian
			# spline (h00)
			twist = self._previous_twist
			self._stop_wait_timer = self._stop_wait_timer + delta_time
			if self._stop_wait_timer > 1.0:
				self._stop_wait_timer = 1.0
			spline_scalar = (1+2*self._stop_wait_timer)*(1-self._stop_wait_timer)*(1-self._stop_wait_timer)
			twist.angular.z = spline_scalar*twist.angular.z
			twist.angular.y = spline_scalar*twist.angular.y
			twist.angular.x = spline_scalar*twist.angular.x
			twist.linear.x = spline_scalar*twist.linear.x
			twist.linear.y = spline_scalar*twist.linear.y
			twist.linear.z = spline_scalar*twist.linear.z

		# Clamp the twist to very small velocities so the wheels have time
		# to steer in the right direction
		if self._new_state_wait_timer > 0:
			twist.angular.z = numpy.clip(twist.angular.z, -0.001, 0.001)
			twist.angular.y = numpy.clip(twist.angular.y, -0.001, 0.001)
			twist.angular.x = numpy.clip(twist.angular.x, -0.001, 0.001)
			twist.linear.x = numpy.clip(twist.linear.x, -0.001, 0.001)
			twist.linear.y = numpy.clip(twist.linear.y, -0.001, 0.001)
			twist.linear.z = numpy.clip(twist.linear.z, -0.001, 0.001)
		else:
			# Clamp the twist values so we don't... kill... anyone....
			twist.angular.z = numpy.clip(twist.angular.z, -0.250, 0.250)
			twist.angular.y = numpy.clip(twist.angular.y, -0.250, 0.250)
			twist.angular.x = numpy.clip(twist.angular.x, -0.250, 0.250)
			twist.linear.x = numpy.clip(twist.linear.x, -0.5, 0.5)
			twist.linear.y = numpy.clip(twist.linear.y, -0.5, 0.5)
			twist.linear.z = numpy.clip(twist.linear.z, -0.5, 0.5)

		# Do not update the previous twist if we are stopping and waiting
		# so we can ramp the twist down slowly to a stop
		if not self._visual_servo_state == VisualServoStates.STOP_AND_WAIT:
			self._previous_twist = twist

		self._publisher.publish(twist)

	def camera_left_info_callback(self, data):
		self._camera_left_image_width = data.width;
		self._camera_left_image_height = data.height;

	def camera_right_info_callback(self, data):
		self._camera_right_image_width = data.width;
		self._camera_right_image_height = data.height;

	def run_visual_servo_action(self, goal):
		update_rate = rospy.Rate(self._loop_frequency)
		while not rospy.is_shutdown():
			self.do_visual_servo()
			if self._has_succeeded:
				self._visual_servo_result.success = True
				self._action_server.set_succeeded(result=self._visual_servo_result)
				break
			else:
				self._action_server.publish_feedback(self._visual_servo_feedback)
			update_rate.sleep()

		# Catch all error handling.  This should not run, but seems good to have
		if self._action_server.is_active():
			self._visual_servo_result.success = False
			self._action_server.set_aborted(self._visual_servo_result, "Shutdown")

		self.reset()

if __name__ == '__main__':
	rospy.init_node('visual_servo')
	visual_servo = VisualServo()
	#visual_servo.run_visual_servo_action(True)
	rospy.spin()
