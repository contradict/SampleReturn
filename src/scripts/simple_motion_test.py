#!/usr/bin/env python
import samplereturn.simple_motion as simple_motion
import math
import rospy
rospy.init_node("simple_motion")
mover = simple_motion.SimpleMover()
