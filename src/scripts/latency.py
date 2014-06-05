#!/usr/bin/env python
"""
Measure platform_motion latency

Send a planned path, measure delay to stitched_path
"""
import sys
import rospy
import numpy as np
from platform_motion_msgs.msg import Path, Knot

def create_path(count):
    p = Path()
    p.header.seq = 0
    p.header.frame_id = 'odom'
    for c in xrange(count):
        k = Knot()
        k.header.seq = c
        k.header.stamp = rospy.Time(0)
        k.header.frame_id = 'base_link'
        k.pose.position.x = float(c)/(count-1)
        k.pose.orientation.w = 1.0
        k.twist.linear.x = 0.5-np.abs(float(c)/(count-1)-0.5)
        p.knots.append(k)
    p.header.stamp = rospy.Time.now()
    return p

rospy.init_node("latency")
p = create_path(10)
sent = p.header.stamp
print_latency = lambda msg, sent = sent: sys.stdout.write( str((rospy.Time.now()- sent).to_sec()) + "\n" )

pub = rospy.Publisher("planned_path", Path)
sub = rospy.Subscriber("stitched_path", Path, print_latency)
rospy.loginfo("Waiting for publishers")
rospy.sleep( 1 )
rospy.loginfo("Publishing %d", pub.get_num_connections())
pub.publish(p)
rospy.loginfo("Waiting for response")
rospy.sleep( 3 )
