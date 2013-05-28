#!/usr/bin/env python
import rospy
import numpy as np

from detection_filter import Filter, Hypothesis
from linemod_detector.msg import NamedPoint

class SampleDetectionFilter(object):
    def __init__(self):
        # node parameters
        self.sample_name = rospy.get_param("~sample_name", "pre_cached")

        self.filter_tolerance = rospy.get_param("~filter_tolerance", 0.1)
        self.filter_alpha = rospy.get_param("~filter_alpha", 0.25)
        self.filter_unsupported_step = rospy.get_param("~filter_unsupported_step", 0.5)
        self.filter_threshold = rospy.get_param("~filter_threshold", 5)
        self.filter_frame = rospy.get_param("~filter_frame", "odom")

        self.filter = Filter(
                Hypothesis(tolerance=self.filter_tolerance,
                           alpha=self.filter_alpha,
                           unsupported_step=self.filter_unsupported_step),
                self.filter_frame)

        rospy.Subscriber('raw_points', NamedPoint, queue_size=1,
                callback=self.handle_point)

        self.publisher = rospy.Publisher('filtered_points', NamedPoint)

    def handle_point(self, msg):
        if msg.name == self.sample_name:
            e = self.filter.update(msg, self.filter_threshold)
            if e is not None:
                fp=NamedPoint()
                fp.header = msg.header
                fp.header.frame_id = self.filter_frame
                fp.point.x = e.position[0]
                fp.point.y = e.position[1]
                fp.point.z = e.position[2]
                fp.name = self.sample_name
                rospy.logdebug("publish: %s", fp)
                self.publisher.publish(fp)

def start_node():
    rospy.init_node('sample_detection_filter', log_level=rospy.DEBUG)
    filter = SampleDetectionFilter()
    rospy.spin()

if __name__=="__main__":
    start_node()
