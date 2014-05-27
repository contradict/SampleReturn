#!/usr/bin/env python
import sys
import rospy
import samplereturn.util as util

from executive import level_one

if __name__=="__main__":
    rospy.init_node('level_one')
    #to allow logging immediately, we must wait for the rosout node to be alive
    util.wait_for_rosout()
    em = level_one.LevelOne()
    rospy.spin()
