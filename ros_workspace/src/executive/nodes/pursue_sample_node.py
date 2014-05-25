#!/usr/bin/env python
import sys
import rospy
import samplereturn.util as util

from executive import pursue_sample

if __name__=="__main__":
    rospy.init_node('pursue_sample')
    #to allow logging immediately, we must wait for the rosout node to be alive
    util.wait_for_rosout()
    em = pursue_sample.PursueSample()
    rospy.spin()
