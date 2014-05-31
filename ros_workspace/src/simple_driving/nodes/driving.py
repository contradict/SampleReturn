#!/usr/bin/env python
"""
Start the SimpleDriving action server
"""
import rospy
from simple_driving import SimpleDriving

if __name__ == "__main__":
    rospy.init_node("driving")
    DRIVE = SimpleDriving()
    rospy.spin()
