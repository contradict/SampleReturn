#!/usr/bin/env python
import sys
import rospy
import samplereturn.util as util

from executive import level_two_star
from executive import level_two_random

if __name__=="__main__":
    rospy.init_node('level_two')
    #to allow logging immediately, we must wait for the rosout node to be alive
    util.wait_for_rosout()
    em = level_two_star.LevelTwoStar()
    rospy.spin()
