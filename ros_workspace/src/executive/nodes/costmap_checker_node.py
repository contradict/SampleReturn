#!/usr/bin/env python
import sys
import rospy
import samplereturn.util as util

from executive import costmap_check

if __name__=="__main__":
    rospy.init_node('costmap_checker')
    #to allow logging immediately, we must wait for the rosout node to be alive
    util.wait_for_rosout()
    checker = costmap_check.ExecutiveCostmapChecker()
    rospy.spin()
