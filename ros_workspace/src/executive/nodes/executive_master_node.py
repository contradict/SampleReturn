#!/usr/bin/env python
import sys
import rospy
import samplereturn.util as util

from executive import executive_master

if __name__=="__main__":
    rospy.init_node('executive_master')
    #to allow logging immediately, we must wait for the rosout node to be alive
    util.wait_for_rosout()
    em = executive_master.ExecutiveMaster()
    rospy.spin()
