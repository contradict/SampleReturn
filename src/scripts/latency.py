#!/usr/bin/env python
"""
Measure platform_motion latency

Send a planned path, measure delay to stitched_path
"""
import sys
import rospy
import numpy as np
from platform_motion_msgs.msg import Path, Knot

global sent

def create_path(count):
    p = Path()
    p.header.seq = 0
    p.header.frame_id = 'odom'
    vel = 0
    vel_new = 0
    x=0
    t=rospy.Time(0)
    dt = 0.100
    for c in xrange(count):
        vel_new = 2.*(0.5-np.abs(float(c)/(count-1)-0.5))
        x += dt*(vel+vel_new)/2.
        vel = vel_new

        k = Knot()
        k.header.seq = c
        k.header.stamp = t
        t += rospy.Duration(dt)
        k.header.frame_id = 'base_link'
        k.pose.position.x = x
        k.pose.orientation.w = 1.0
        k.twist.linear.x = vel_new
        p.knots.append(k)
    p.header.stamp = rospy.Time.now()
    return p

def print_latency(msg):
    global sent
    rospy.loginfo("Latency: %f", (rospy.Time.now()-sent).to_sec())


rospy.init_node("latency")
p = create_path(10)

pub = rospy.Publisher("planned_path", Path)
sub = rospy.Subscriber("stitched_path", Path, print_latency)
rospy.loginfo("Waiting for publishers")
rospy.sleep( 1 )

for k in range(1):
    rospy.loginfo("Publishing %d", pub.get_num_connections())
    sent = rospy.Time.now()
    pub.publish(p)
    rospy.loginfo("Waiting for response")
    rospy.sleep(0.5)
