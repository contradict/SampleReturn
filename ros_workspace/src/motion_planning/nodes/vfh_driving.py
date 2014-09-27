#!/usr/bin/env python
"""
Start the SimpleDriving action server
"""
import rospy
from motion_planning import vfh_move_server

if __name__ == "__main__":
    rospy.init_node("vfh_mover")
    MOVER = vfh_move_server.VFHMoveServer()
    rospy.spin()
