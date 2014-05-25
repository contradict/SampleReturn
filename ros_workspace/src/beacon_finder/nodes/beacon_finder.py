#!/usr/bin/env python
import sys
import rospy
import rosnode
import math
import cv2
import numpy
import tf
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Vector3, Quaternion, PoseStamped
from tf.transformations import quaternion_from_matrix

class BeaconFinder:
	"""A class to locate the beacon in an image and publish a vector in the camera's frame from the robot to the beacon"""
	
	def __init__(self):
		self._image_subscriber = rospy.Subscriber('camera_image', Image, self.image_callback, None, 8)
		self._camera_info_subscriber = rospy.Subscriber('camera_info', CameraInfo, self.camera_info_callback, None, 8)
		self._beacon_pose_publisher = rospy.Publisher('beacon_pose', PoseStamped)
		self._beacon_debug_image = rospy.Publisher('beacon_debug_img', Image)
		self._cv_bridge = CvBridge()
		
		#tf broadcaster stuff
		self.tf_broadcaster = tf.TransformBroadcaster()
		rospy.Timer(rospy.Duration(0.5), self.broadcast_beacon_tf) 

		# Get params
		self._num_rows = rospy.get_param("~num_rows", 3)
		self._num_columns = rospy.get_param("~num_columns", 9)
		self._blob_color = rospy.get_param("~blob_color", 0)
		self._blob_min_area = rospy.get_param("~blob_min_area", 9)
		self._blob_max_area = rospy.get_param("~blob_max_area", 5000)
		self._blob_min_threshold = rospy.get_param("~blob_min_threshold", 20)
		self._blob_max_threshold = rospy.get_param("~blob_max_threshold", 220)
		self._blob_threshold_step = rospy.get_param("~blob_threshold_step", 10)
		self._blob_min_distance_between_blobs = rospy.get_param("~blob_min_distance_between_blobs", 3.0)
		self._blob_repeatability = rospy.get_param("~blob_repeatability", 3L)
		self._do_histogram_equalization = rospy.get_param("~do_histogram_equalization", False)

		# Initialize member variables
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
		self._camera_matrix = numpy.array([0])

		if self._do_histogram_equalization:
			self._image_output_encoding = '8UC1'
		else:
			self._image_output_encoding = 'bgr8'

		# Construct a matrix of points representing the circles grid facing the camera
		d = rospy.get_param("~vertical_distance_between_circles", 0.3048)
		if rospy.get_param("~is_first_column_shifted_down", False):
			offset = -d/2.0
		else:
			offset = 0.0
		self._circles_grid = numpy.array([0,0,0], dtype="float32") # Create a 'dummy' row
		for column in range(-self._num_columns/2+1, self._num_columns/2+1):
			for row in range(0, -self._num_rows, -1):
				self._circles_grid = numpy.vstack((self._circles_grid, numpy.array([column*d/2.0,row*d+offset,0.0])))
			if offset == 0.0:
				offset = -d/2.0
			else:
				offset = 0.0
		self._circles_grid = numpy.delete(self._circles_grid, 0, 0) # Delete the 'dummy' row
		self._circles_grid = numpy.array(self._circles_grid, dtype="float32") # Ensure float32 data type, which shouldn't be necessary at this point, but it is...

	def image_callback(self, image):
		# This function receives an image, attempts to locate the beacon
		# in it, then if successful outputs a vector towards it in the
		# camera's frame
		if self._camera_matrix.any():
			image_cv = numpy.asarray(self._cv_bridge.imgmsg_to_cv(image, 'bgr8'))

			if self._do_histogram_equalization:
				image_cv = cv2.cvtColor(image_cv, cv2.COLOR_RGB2GRAY)
				image_cv = cv2.equalizeHist(image_cv)

			found_beacon, centers = cv2.findCirclesGrid(image_cv, (self._num_rows, self._num_columns), flags=cv2.CALIB_CB_ASYMMETRIC_GRID, blobDetector=self._blob_detector)

			if found_beacon:
				beacon_pose = PoseStamped()

				# Compute the position and orientation of the beacon
				rotation_vector, translation_vector, inliers = cv2.solvePnPRansac(self._circles_grid, centers, self._camera_matrix, self._distortion_coefficients)
				rotation_matrix = cv2.Rodrigues(rotation_vector)[0]
				rotation_matrix = numpy.hstack((rotation_matrix, numpy.array([[0],[0],[0]])))
				rotation_matrix = numpy.vstack((rotation_matrix, numpy.array([0,0,0,1])))
				quat = quaternion_from_matrix(rotation_matrix)

				beacon_pose.pose.position.x = translation_vector[0]
				beacon_pose.pose.position.y = translation_vector[1]
				beacon_pose.pose.position.z = translation_vector[2]
				beacon_pose.pose.orientation.x = quat[0]
				beacon_pose.pose.orientation.y = quat[1]
				beacon_pose.pose.orientation.z = quat[2]
				beacon_pose.pose.orientation.w = quat[3]
				beacon_pose.header = image.header
				self._beacon_pose_publisher.publish(beacon_pose)

				# Publish debug beacon image with the beacon points circled if there are subscribers
				if self._beacon_debug_image.get_num_connections() > 0:
					cv2.drawChessboardCorners(image_cv, (self._num_rows, self._num_columns), centers, found_beacon)
					self._beacon_debug_image.publish(self._cv_bridge.cv_to_imgmsg(cv2.cv.fromarray(image_cv), self._image_output_encoding))
			else:
				# Publish debug beacon image if there are subscribers
				if self._beacon_debug_image.get_num_connections() > 0:
					self._beacon_debug_image.publish(self._cv_bridge.cv_to_imgmsg(cv2.cv.fromarray(image_cv), self._image_output_encoding))

	def camera_info_callback(self, camera_info):
		self._camera_matrix = numpy.array( [ \
										[camera_info.K[0], 0, camera_info.K[2]], \
										[0, camera_info.K[4], camera_info.K[5]], \
										[0, 0, 1]])
		self._distortion_coefficients = camera_info.D
		
	def broadcast_beacon_tf():
		now = rospy.Time.now()
		beacon_trans = (0.0, -1.3, -1.0)
		beacon_rot = tf.transformations.quaternion_from_euler(-math.pi/2,
								      0.0,
								      -math.pi/2,
								      'rxyz')
		self.tf_broadcaster.sendTransform(beacon_trans,
						  beacon_rot,
						  now,
						  '/beacon',
						  '/platform')
	
		zero_translation = (0,0,0)
		zero_rotation = (0,0,0,1)
		self.tf_broadcaster.sendTransform(zero_translation,
						  zero_rotation,
						  now,
						  '/platform',
						  '/map')

if __name__ == '__main__':
	rospy.init_node('beacon_finder')
	beacon_finder = BeaconFinder()
	rospy.spin()
