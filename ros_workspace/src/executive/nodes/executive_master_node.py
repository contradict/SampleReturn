#!/usr/bin/env python
import sys
import rospy
import rosnode

from executive import executive_master

if __name__=="__main__":
    rospy.init_node('executive_master')
    em = executive_master.ExecutiveMaster()
    #em.start()
    rospy.spin()
