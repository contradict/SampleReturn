#!/usr/bin/env python
import sys
import rospy
import rosnode
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Vector3

class BeaconFinder:
	"""A class to locate the beacon in an image and publish a vector in the camera's frame from the robot to the beacon"""
	
	def __init__(self):
		self._image_subscriber = rospy.Subscriber('mono_img', Image, self.image_callback, None, 1)
		self._camera_info_subscriber = rospy.Subscriber('cam_info', CameraInfo, self.camera_info_callback, None, 1)
		self._beacon_vector_publisher = rospy.Publisher('/beacon_vector', Vector3)
		self.cv_bridge = CvBridge()

		# Get params
		self._num_rows = rospy.get_param("~num_rows", 4)
		self._num_columns = rospy.get_param("~num_columns", 11)
		self._corner_circles_vertical_distance_meters = rospy.get_param("~corner_circles_vertical_distance_meters", 0.5)
		self._camera_image_sensor_width_mmeters = rospy.get_param("~camera_image_sensor_width_mmeters", 15.6)
		self._camera_image_sensor_height_mmeters = rospy.get_param("~camera_image_sensor_height_mmeters", 23.6)

		# Initialize member variables
		self._camera_pixel_width = None
		self._camera_pixel_height = None
		self._camera_hfov = None
		self._camera_vfov = None
		self._camera_focal_length = None

	def image_callback(self, image):
		# This function receives an image, attempts to locate the beacon
		# in it, then if successful outputs a vector towards it in the
		# camera's frame
		if self._camera_focal_length:
			image_cv = self.cv_bridge.imgmsg_to_cv(image, 'bgr8')
			found_beacon, centers, cv2.findCirclesInImage(image_cv, (self._num_rows, self._num_columns), flags=cv2.CALIB_CB_ASYMMETRIC_GRID)

			if found_beacon:
				# Compute the approximate distance to the beacon
				corner_circles_vector_pixels = centers[0][0] - centers[1][0]
				corner_circles_vertical_distance_pixels = math.hypot(*corner_circles_vector_pixels)
				approx_beacon_distance_meters = self._camera_focal_length*self._corner_circles_vertical_distance_meters/(corner_circles_vertical_distance_pixels*self._camera_image_sensor_height_mmeters/self._camera_pixel_height/1000)

				# Generate a vector pointing towards the beacon
				beacon_center_pixel = centers[2][0]
				beacon_center_angle_yaw = (beacon_center_pixel.x/self._camera_pixel_width-0.5)*self._camera_hfov
				beacon_center_angle_pitch = (beacon_center_pixel.y/self._camera_pixel_height-0.5)*self._camera_vfov
				beacon_vector = Vector3(1,0,0)
				beacon_vector.y = math.tan(beacon_center_angle_yaw)
				beacon_vector.z = math.tan(beacon_center_angle_pitch)
				beacon_vector_magnitude = math.sqrt(beacon_vector.y*beacon_vector.y + beacon_vector.z*beacon_vector.z + 1)
				beacon_vector.x = beacon_vector.x*approx_beacon_distance_meters/beacon_vector_magnitude
				beacon_vector.y = beacon_vector.y*approx_beacon_distance_meters/beacon_vector_magnitude
				beacon_vector.z = beacon_vector.z*approx_beacon_distance_meters/beacon_vector_magnitude
				self._beacon_vector_publisher.publish(beacon_vector)

	def camera_info_callback(self, camera_info):
		self._camera_pixel_width = camera_info.width
		self._camera_pixel_height = camera_info.height
		self._camera_focal_length = camera_info.K[0]
		self._camera_hfov = 2*math.atan(0.5*self._camera_image_sensor_width_mmeters/camera_info.K[0])
		self._camera_vfov = 2*math.atan(0.5*self._camera_image_sensor_height_mmeters/camera_info.K[4])

if __name__ == '__main__':
	rospy.init_node('beacon_finder')
	beacon_finder = BeaconFinder()
	rospy.spin()
