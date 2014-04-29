#!/usr/bin/env python
import pylab
import numpy as np
from numpy import arctan2

import rospy

import platform_motion_msgs.msg as plat_msgs

global gmsg

def drawpath(msg):
    fig=pylab.figure(1)
    fig.clear()
    ax=fig.add_subplot(111)

    pts=np.array([(knot.pose.position.x, knot.pose.position.y) for knot in msg.knots])
    ax.plot( pts[:,0], pts[:,1] )
    fig.canvas.draw()

def makesub(topicname, topicmsg):
    msgs=[]
    def savemsg(m):
        msgs.append(m)
    return msgs,rospy.Subscriber(topicname, topicmsg, savemsg)

def stitched():
    return makesub("/motion/stitched_path", plat_msgs.Path)

def planned():
    return makesub("/motion/planned_path", plat_msgs.Path)

def msg2path(m):
    pts=np.array([(knot.pose.position.x,
                   knot.pose.position.y,
                   2*arctan2(knot.pose.orientation.z,knot.pose.orientation.w),
                   knot.twist.linear.x,
                   knot.twist.linear.y,
                   knot.twist.angular.z,
                   rospy.Time(knot.header.stamp.secs, knot.header.stamp.nsecs).to_sec()
                   ) for knot in m.knots])
    return pts

