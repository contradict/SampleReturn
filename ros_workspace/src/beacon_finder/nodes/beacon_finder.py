#!/usr/bin/env python
import sys
import rospy
import rosnode
import math
import cv2
import numpy
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Vector3

class BeaconFinder:
	"""A class to locate the beacon in an image and publish a vector in the camera's frame from the robot to the beacon"""
	
	def __init__(self):
		self._image_subscriber = rospy.Subscriber('cam_img', Image, self.image_callback, None, 1)
		self._camera_info_subscriber = rospy.Subscriber('cam_info', CameraInfo, self.camera_info_callback, None, 1)
		self._beacon_vector_publisher = rospy.Publisher('/beacon_vector', Vector3)
		self._beacon_debug_image = rospy.Publisher('/beacon_debug_img', Image)
		self._cv_bridge = CvBridge()

		# Get params
		self._num_rows = rospy.get_param("~num_rows", 4)
		self._num_columns = rospy.get_param("~num_columns", 11)
		self._corner_circles_vertical_distance_meters = rospy.get_param("~corner_circles_vertical_distance_meters", 0.6895)
		self._camera_image_sensor_width_mmeters = rospy.get_param("~camera_image_sensor_width_mmeters", 15.6)
		self._camera_image_sensor_height_mmeters = rospy.get_param("~camera_image_sensor_height_mmeters", 23.6)
		self._blob_color = rospy.get_param("~blob_color", 0)
		self._blob_min_area = rospy.get_param("~blob_min_area", 9)
		self._blob_max_area = rospy.get_param("~blob_max_area", 5000)
		self._blob_min_threshold = rospy.get_param("~blob_min_threshold", 20)
		self._blob_max_threshold = rospy.get_param("~blob_max_threshold", 220)
		self._blob_threshold_step = rospy.get_param("~blob_threshold_step", 10)
		self._blob_min_distance_between_blobs = rospy.get_param("~blob_min_distance_between_blobs", 3.0)
		self._blob_repeatability = rospy.get_param("~blob_repeatability", 3L)
		self._do_histogram_equalization = rospy.get_param("~do_histogram_equalization", False)
		self._do_publish_debug = rospy.get_param("~do_publish_debug", True)

		# Initialize member variables
		self._camera_pixel_width = None
		self._camera_pixel_height = None
		self._camera_hfov = None
		self._camera_vfov = None
		self._camera_focal_length = None
		self._blob_detector_params = cv2.SimpleBlobDetector_Params()
		self._blob_detector_params.blobColor = self._blob_color
		self._blob_detector_params.minArea = self._blob_min_area
		self._blob_detector_params.maxArea = self._blob_max_area
		self._blob_detector_params.minThreshold = self._blob_min_threshold
		self._blob_detector_params.maxThreshold = self._blob_max_threshold
		self._blob_detector_params.thresholdStep = self._blob_threshold_step
		self._blob_detector_params.minDistBetweenBlobs = self._blob_min_distance_between_blobs
		self._blob_detector_params.minRepeatability = self._blob_repeatability
		self._blob_detector = cv2.SimpleBlobDetector(self._blob_detector_params)

		if self._do_histogram_equalization:
			self._image_output_encoding = '8UC1'
		else:
			self._image_output_encoding = 'bgr8'

	def image_callback(self, image):
		# This function receives an image, attempts to locate the beacon
		# in it, then if successful outputs a vector towards it in the
		# camera's frame
		if self._camera_focal_length:
			image_cv = numpy.asarray(self._cv_bridge.imgmsg_to_cv(image, 'bgr8'))
			if self._do_histogram_equalization:
				image_cv = cv2.cvtColor(image_cv, cv2.COLOR_RGB2GRAY)
				image_cv = cv2.equalizeHist(image_cv)
			found_beacon, centers = cv2.findCirclesGrid(image_cv, (self._num_rows, self._num_columns), flags=cv2.CALIB_CB_ASYMMETRIC_GRID, blobDetector=self._blob_detector)

			if found_beacon:
				if self._do_publish_debug:
					# Publish debug beacon image
					cv2.drawChessboardCorners(image_cv, (self._num_rows, self._num_columns), centers, found_beacon)
					self._beacon_debug_image.publish(self._cv_bridge.cv_to_imgmsg(cv2.cv.fromarray(image_cv), self._image_output_encoding))

				# Compute the approximate distance to the beacon
				lower_left_circle_index = 0
				top_left_circle_index = self._num_rows-1
				corner_circles_vector_pixel_x = centers[lower_left_circle_index][0][0] - centers[top_left_circle_index][0][0]
				corner_circles_vector_pixel_y = centers[lower_left_circle_index][0][1] - centers[top_left_circle_index][0][1]
				corner_circles_vertical_distance_pixels = math.hypot(corner_circles_vector_pixel_x, corner_circles_vector_pixel_y)
				approx_beacon_distance_meters = self._camera_focal_length*self._corner_circles_vertical_distance_meters/(corner_circles_vertical_distance_pixels*self._camera_image_sensor_height_mmeters/self._camera_pixel_height)

				# Generate a vector pointing towards the beacon
				beacon_center_pixel = centers[self._num_rows*(self._num_columns/2)+self._num_rows/2][0]
				beacon_center_angle_yaw = (beacon_center_pixel[0]/self._camera_pixel_width-0.5)*self._camera_hfov
				beacon_center_angle_pitch = -(beacon_center_pixel[1]/self._camera_pixel_height-0.5)*self._camera_vfov
				beacon_vector = Vector3(0,0,1)
				beacon_vector.x = math.tan(beacon_center_angle_yaw)
				beacon_vector.y = math.tan(beacon_center_angle_pitch)
				beacon_vector_magnitude = math.sqrt(beacon_vector.x*beacon_vector.x + beacon_vector.y*beacon_vector.y + 1)
				beacon_vector.x = beacon_vector.x*approx_beacon_distance_meters/beacon_vector_magnitude
				beacon_vector.y = beacon_vector.y*approx_beacon_distance_meters/beacon_vector_magnitude
				beacon_vector.z = beacon_vector.z*approx_beacon_distance_meters/beacon_vector_magnitude
				self._beacon_vector_publisher.publish(beacon_vector)
			else:
				if self._do_publish_debug:
					self._beacon_debug_image.publish(self._cv_bridge.cv_to_imgmsg(cv2.cv.fromarray(image_cv), self._image_output_encoding))

	def camera_info_callback(self, camera_info):
		self._camera_pixel_width = 3696
		self._camera_pixel_height = 2448
		self._camera_focal_length = 18
		self._camera_hfov = 2*math.atan(0.5*self._camera_image_sensor_width_mmeters/18)
		self._camera_vfov = 2*math.atan(0.5*self._camera_image_sensor_height_mmeters/18)
		#self._camera_pixel_width = camera_info.width
		#self._camera_pixel_height = camera_info.height
		#self._camera_focal_length = camera_info.K[0]
		#self._camera_hfov = 2*math.atan(0.5*self._camera_image_sensor_width_mmeters/camera_info.K[0])
		#self._camera_vfov = 2*math.atan(0.5*self._camera_image_sensor_height_mmeters/camera_info.K[4])

if __name__ == '__main__':
	rospy.init_node('beacon_finder')
	beacon_finder = BeaconFinder()
	rospy.spin()
