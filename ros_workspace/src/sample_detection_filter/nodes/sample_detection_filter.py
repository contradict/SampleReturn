#!/usr/bin/env python
import sys
import math
import numpy
import rospy
import rosnode
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Point

class SampleDetectionFilter:
	"""A class to filter detected sample locations to help stablize them spatially"""
	
	def __init__(self):
		self._number_samples = rospy.get_param('~number_samples', 5)
		self._weighting_factor = rospy.get_param('~weighting_factor', 1.0)
		self._reset_filter_seconds = rospy.get_param('~reset_filter_seconds', 10.0)
		self._samples = []

		# TODO - Add a single subscriber for the NamedPoints
		self._point_subscriber = rospy.Subscriber('/red_puck_imgpoints', PointStamped, self.object_point_callback, None, 1)
		self._filtered_point_publisher = rospy.Publisher('/red_puck_imgpoints_filtered', PointStamped)

	def object_point_callback(self, data):
		# This function receives a centroid location of an object
		# in pixel space and does the filtering 
		if len(self._samples) == 0 or (data.header.stamp-self._samples[0].header.stamp).to_sec() > self._reset_filter_seconds:
			self._samples = self._number_samples*[data]
		else:
			self._samples.insert(0, data)
			self._samples.pop()
		
		
		self._filtered_point_publisher.publish(self.average_samples())

	def average_samples(self):
		delta_time = []
		for sample in self._samples:
			delta_time.append((self._samples[0].header.stamp-sample.header.stamp).to_sec())

		print(delta_time)
		sum_weights = 0
		average_point = Point(0,0,0)
		for i in range(self._number_samples):
			weight = math.exp(-self._weighting_factor*delta_time[i])
			sum_weights = sum_weights + weight
			weighted_point = self._samples[i].point
			weighted_point.x = weighted_point.x * weight
			weighted_point.y = weighted_point.y * weight
			weighted_point.z = weighted_point.z * weight
			average_point.x = average_point.x + weighted_point.x
			average_point.y = average_point.y + weighted_point.y
			average_point.z = average_point.z + weighted_point.z
		average_point.x = average_point.x / sum_weights
		average_point.y = average_point.y / sum_weights
		average_point.z = average_point.z / sum_weights

		point_stamped = PointStamped()
		point_stamped.point = average_point
		point_stamped.header.stamp = self._samples[0].header.stamp
		return point_stamped

if __name__ == '__main__':
	rospy.init_node('sample_detection_filter')
	sample_detection_filter = SampleDetectionFilter()
	rospy.spin()
