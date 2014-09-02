#!/usr/bin/env python
"""
Start the SimpleDriving action server
"""
import rospy
from motion_planning import SimpleMoveServer

if __name__ == "__main__":
    rospy.init_node("simple_mover")
    MOVER = SimpleMoveServer()
    rospy.spin()
