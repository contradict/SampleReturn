#!/usr/bin/env python
"""
Start the SimpleDriving action server
"""
import rospy
from motion_planning import simple_move_server

if __name__ == "__main__":
    rospy.init_node("simple_mover")
    MOVER = simple_move_server.SimpleMoveServer()
    rospy.spin()
