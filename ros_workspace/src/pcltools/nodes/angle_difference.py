#!/usr/bin/env python

import rospy
from pcl_msgs.msg import ModelCoefficients
import numpy as np
from numpy.linalg import norm
from scipy.optimize import leastsq
from message_filters import ApproximateTimeSynchronizer, Subscriber
import tf2_ros
from tf.transformations import quaternion_multiply as qmpy
from tf.transformations import quaternion_about_axis as qaxis
from tf.transformations import quaternion_inverse as qinv
from tf.transformations import euler_from_quaternion

extract_vector = lambda msg: np.array(msg.values[:3])/norm(msg.values[:3])
angle_error = lambda vec, ref: np.degrees(np.arccos(np.dot(ref, vec)))
axiserror = lambda vec, ax, ap: np.degrees(np.arctan(vec[ax]/np.sqrt(vec[ax]**2 + vec[ap]**2)))

def qfromtwo(start, end):
    vect = np.cross(start, end)
    n = norm(vect)
    return np.r_[np.sqrt(1.0-n**2), vect]

def solveq(ground, qg, qw):
    errexpr = lambda r, qg, qw: qmpy(r, qmpy(qg, qinv(qw))) - np.r_[1.0, 0.0, 0.0, 0.0]
    minexpr = lambda theta: errexpr(qaxis(theta, ground), qg, qw)
    theta, _ = leastsq(minexpr, 0.0 )
    qerror = qmpy(qaxis(theta, ground), qg)
    return qerror

def callback(*msgs):
    global qs, qp
    cg, pg, sg, cw, pw, sw = map(extract_vector, msgs)
    gerr = lambda vec: (angle_error(cg, vec), axiserror(vec, 0, 2), axiserror(vec, 1, 2))
    werr = lambda vec: (angle_error(cw, vec), axiserror(vec, 1, 0), axiserror(vec, 2, 0))
    rospy.loginfo("Port Ground Error: %5.3f, %5.3f, %5.3f"%gerr(pg))
    rospy.loginfo("Port Wall Error:   %5.3f, %5.3f, %5.3f"%werr(pw))
    rospy.loginfo("Starboard Ground Error: %5.3f, %5.3f, %5.3f"%gerr(sg))
    rospy.loginfo("Starboard Wall Error:   %5.3f, %5.3f, %5.3f"%werr(sw))

    qmeas_pg = qfromtwo(cg, pg)
    qmeas_pw = qfromtwo(cw, pw)
    port_qerror = qmpy(qinv(qp), qmpy(solveq(cg, qmeas_pg, qmeas_pw), qp))
    r, p, y = euler_from_quaternion(port_qerror)
    rospy.loginfo("Port error RPY: (%5.4f, %5.4f, %5.4f)", r, p, y)

    qmeas_sg = qfromtwo(cg, sg)
    qmeas_sw = qfromtwo(cw, sw)
    starboard_qerror = qmpy(qinv(qs), qmpy(solveq(cg, qmeas_sg, qmeas_sw), qs))
    r, p, y = euler_from_quaternion(starboard_qerror)
    rospy.loginfo("Starboard error RPY: (%5.4f, %5.4f, %5.4f)", r, p, y)


rospy.init_node("angle_difference")
topics = ["%s_%s_coefficients"%(direction, plane)
        for plane in ["ground", "wall"]
        for direction in ["center", "port", "starboard"]
        ]
rospy.loginfo("topics: %s", topics)
subscribers = map(
        lambda topic: Subscriber(topic, ModelCoefficients, queue_size=2),
        topics)

buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buffer)
tport = buffer.lookup_transform('navigation_port_left_camera', 'base_link',
        rospy.Time(0), timeout=rospy.Duration(10))
qpm = tport.transform.rotation
qp = np.r_[qpm.w, qpm.x, qpm.y, qpm.z]
tstar = buffer.lookup_transform('navigation_starboard_left_camera', 'base_link',
        rospy.Time(0), timeout=rospy.Duration(10))
qsm = tstar.transform.rotation
qs = np.r_[qsm.w, qsm.x, qsm.y, qsm.z]

sync = ApproximateTimeSynchronizer(subscribers, 10, slop=0.1)
sync.registerCallback(callback)

rospy.spin()
