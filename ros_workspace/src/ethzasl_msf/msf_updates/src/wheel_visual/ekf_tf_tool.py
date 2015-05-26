#!/usr/bin/env python
import rospy
import tf2_ros
import tf
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped, Transform
from std_msgs.msg import Header
from sensor_fusion_comm.msg import DoubleArrayStamped
from tf.transformations import quaternion_from_euler as rpy
from tf.transformations import euler_from_quaternion, compose_matrix, decompose_matrix
from math import pi, radians
from numpy.linalg import inv
from numpy import dot, ndarray
from math import sin, cos

def eq(q):
    return euler_from_quaternion((q.x, q.y, q.z, q.w))

class FancyTransform(TransformStamped):
    def __init__(self, *args, **kwargs):
        if (len(args) == 3 and
            isinstance(args[0], Header) and
            isinstance(args[1], str) and
            isinstance(args[2], ndarray) and
            args[2].shape == (4, 4)):
            _, _, ang, trans, _ = decompose_matrix(args[2])
            super(FancyTransform, self).__init__(
                    args[0],
                    args[1],
                    Transform(Vector3(*trans),
                              Quaternion(*rpy(*ang))))
        elif (len(args) == 1 and isinstance(args[0], TransformStamped)):
            super(FancyTransform, self).__init__(args[0].header,
                    args[0].child_frame_id,
                    args[0].transform)
        else:
            super(FancyTransform, self).__init__(*args, **kwargs)

    def mat(self):
        return compose_matrix(
                angles=eq(self.transform.rotation),
                translate=(self.transform.translation.x,
                           self.transform.translation.y,
                           self.transform.translation.z))

    def inv(self):
        _, _, iang, itrans, _ = decompose_matrix(inv(self.mat()))
        return FancyTransform(
                Header(self.header.seq,
                       self.header.stamp,
                       self.child_frame_id),
                self.header.frame_id,
                Transform(Vector3(*itrans),
                          Quaternion(*rpy(*iang))))

    def __mul__(self, other):
        if isinstance(other, TransformStamped):
            other = FancyTransform(other.header, other.child_frame_id,
                    other.transform)
        if isinstance(other, FancyTransform):
            return FancyTransform(
                    other.header,
                    self.child_frame_id,
                    dot(self.mat(), other.mat()))
        elif isinstance(other, Vector3):
            v3 = dot(self.mat(), (other.x, other.y, other.z, 1.0))
            return Vector3(*v3[:3])
        raise ValueError("other is not valid type")

class EKFTransformTool(object):
    def __init__(self):
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()

        # inertial sensor is 0.4m above base_link
        inertial_above_base = 0.4
        # and starts out at (10,10) in x,y
        self.p_iw = Vector3(3, 3, inertial_above_base)
        # inertial frame is based on NED, so align x-axis and rotate around it
        self.q_iw = Quaternion(*rpy(pi, 0, 0))
        # initial velocity is 1m/s xhat
        self.v_iw = Vector3(1.0, 0.0, 0.0)
        # initial angular rate is pure yaw at 0.1rad/s
        self.omega_i = Vector3(0.0, 0.0, 0.1)

        # base_link relative to inertial sensor
        # base_link is forward and down (down is +Z in inertial)
        self.p_bi = Vector3(0.3, 0, inertial_above_base)
        # Z is up in base_link, undo inertial rotation
        self.q_bi = Quaternion(*rpy(-pi, 0, 0))

        # cameras relative to inertial frame.
        # forward (+x) left (-y) and up (-z)
        self.p_ci = Vector3(0.45, -0.05, -1.0)
        # cameras rotate to place zhat along optical axis and yhat downish
        self.q_ci = Quaternion(*rpy(radians(60), 0, radians(90)))

        # odometry drift expressed as vector in odometry frame to world origin
        self.p_wo = Vector3(-1, -5, 0)
        self.q_wo = Quaternion(*rpy(0, 0, 0.5))

        # vision drift expressed as vector in vision frame to world origin
        self.p_wv = Vector3(-2, -2, 0)
        self.q_wv = Quaternion(*rpy(0.1, 0.1, 0.1))

    def __getattr__(self, transform):
        if transform[:2] != "T_":
            raise AttributeError
        frames = transform.split("_")[1]
        frames_inv = frames[::-1]
        if hasattr(self, "q_"+frames) and hasattr(self, "p_"+frames):
            now = rospy.Time.now()
            q = getattr(self, "q_"+frames)
            p = getattr(self, "p_"+frames)
            return FancyTransform(Header(0, now, frames[1]),
                                  frames[0],
                                  Transform(p, q))
        elif hasattr(self, "q_"+frames_inv) and hasattr(self, "p_"+frames_inv):
            return getattr(self, "T_"+frames_inv).inv()
        else:
            raise AttributeError

    def send(self):
        now = rospy.Time.now()
        transforms = []
        def addXform(frame, child, translation, rotation):
            transforms.append(
                    TransformStamped(Header(0, now, frame),
                                     child,
                                     Transform(translation,
                                         rotation)))
        def addXformInv(frame, child, translation, rotation):
            _, _, iang, itrans, _ = decompose_matrix(
                inv(compose_matrix(
                    angles=eq(rotation),
                    translate=(translation.x, translation.y, translation.z))))
            addXform(frame, child,
                    Vector3(*itrans),
                    Quaternion(*rpy(*iang)))
        addXform("map", "inertial", self.p_iw, self.q_iw)
        addXformInv("map", "odometry", self.p_wo, self.q_wo)
        addXformInv("map", "vision", self.p_wv, self.q_wv)
        addXform("inertial", "camera", self.p_ci, self.q_ci)
        addXform("inertial", "base", self.p_bi, self.q_bi)
        self.tf_broadcaster.sendTransform(transforms)

def TfromSt(h, chld, st):
    return FancyTransform(h, chld,
            Transform(Vector3(*st[4:7]),
               Quaternion(-st[1], -st[2], -st[3], st[0])))

#  p,      0:3
#  v,      3:6
#  q,      6:10
#  b_w,    10:13
#  b_a,    13:16
#  L,      16:17
#  q_wv,   17:21
#  p_wv,   21:24
#  q_ic,   24:28
#  p_ic,   28:31
#  yaw_d, 31:32
#  p_d,   32:34
#  q_ib,   34:38
#  p_ib,   38:41
class StateToTF(object):
    def __init__(self):
        self.b_ = tf2_ros.TransformBroadcaster()

        self.state_sub_ = rospy.Subscriber("state", DoubleArrayStamped,
                self.handle_state)

    def handle_state(self, msg):
        # position transform
        # Vector specified in world frame, location of imu frame origin
        # Quaternion applied to vector in world frame gives vector in imu frame
        # tf uses opposite convention for rotations
        h = Header(msg.header.seq, msg.header.stamp, 'map')
        t_iw = FancyTransform(h, 'imu', Transform(
                Vector3(*msg.data[:3]),
                Quaternion(-msg.data[7], -msg.data[8], -msg.data[9], msg.data[6])))
        rospy.loginfo("t_iw:\n%s", t_iw)

        # visual odometry drift transform
        # Vector specified in world frame
        # Quaternion as for position transform
        h = Header(msg.header.seq, msg.header.stamp, 'map')
        t_wv = FancyTransform(h, 'vision', Transform(
                Vector3(*msg.data[21:24]),
                Quaternion(-msg.data[18], -msg.data[19], -msg.data[20], msg.data[17])))
        rospy.loginfo("t_wv:\n%s", t_wv)

        # camera location transform
        # vector specified in imu frame
        # quaternion rotates vector in camera frame to imu frame
        # for tf tree consistency, emit imu->camera transform
        # vector is correct, since tf uses inverse convention, no quaternion
        # conjugation necessary
        h = Header(msg.header.seq, msg.header.stamp, 'imu')
        t_ic = FancyTransform(h, 'camera', Transform(
                Vector3(*msg.data[28:31]),
                Quaternion(msg.data[25], msg.data[26], msg.data[27], msg.data[24])))
        rospy.loginfo("t_ic:\n%s", t_ic)

        # odometry yaw drift
        # same as vision frame drift (quaternion conjugation necessary)
        h = Header(msg.header.seq, msg.header.stamp, 'map')
        y_wo = msg.data[31]
        t_wo = FancyTransform(h, 'odometry', Transform(
                Vector3(msg.data[32], msg.data[33], 0),
                Quaternion(0.0, 0.0, -sin(y_wo/2.0), cos(y_wo/2.0))))
        rospy.loginfo("t_wo:\n%s", t_wo)

        # odometry sensor position
        # same as camera position
        h = Header(msg.header.seq, msg.header.stamp, 'imu')
        t_ib = FancyTransform(h, 'base', Transform(
                Vector3(*msg.data[38:41]),
                Quaternion(msg.data[35], msg.data[36], msg.data[37], msg.data[34])))
        rospy.loginfo("t_ib:\n%s", t_ib)

        self.b_.sendTransform([t_iw, t_wv, t_ic, t_wo, t_ib])

def run_node():
    rospy.init_node("ekf_tf_tool")
    #tool = EKFTransformTool()
    #rospy.Timer(rospy.Duration(0.05),
    #        lambda evt: tool.send())
    tool = StateToTF()
    return tool

if __name__ == "__main__":
    run_node()
    rospy.spin()

