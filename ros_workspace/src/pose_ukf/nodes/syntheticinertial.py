#!/usr/bin/env python
""" Produce synthetic IMU data for simple spinning and wobbling.
"""
from __future__ import print_function
import time
import termios
import tty
import select
import os
import sys

import numpy as np
from numpy import cos, pi, sqrt, arctan2

import rospy
from geometry import rotations, SO3
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion, Pose, PoseStamped
from std_msgs.msg import Header
from rosgraph_msgs.msg import Clock

Omega = np.r_[[0.15],
              [0.00],
              [0.1]]

Omegaf = np.r_[[0.25],
               [0.0],
               [0.0]]

Gravity = np.r_[[[ 0.0],
                 [ 0.0],
                 [ 9.8066]]]


def integrate(dt, phase, orientation):
    """ Integrate motion to produce new orientation """
    phase += 2*pi*dt*Omegaf
    omega = Omega*cos(phase)
    delta = (np.eye(3), rotations.hat_map(dt*omega))
    orientation = SO3.multiply(orientation, SO3.expmap(delta))
    return phase, omega, orientation

def accelerometer(orientation):
    """ Produce gravity vector from orientation matrix """
    acceleration = SO3.multiply(SO3.inverse(orientation), Gravity)
    return acceleration

def rpy(orientation):
    """ Roll, Pitch, Yaw from rotation matrix """
    c2c3 = orientation[0, 0]
    mc2s3 = orientation[0, 1]
    s2 = orientation[0, 2]
    mc2s1 = orientation[1, 2]
    c1c2 = orientation[2, 2]
    c2 = sqrt((c2c3**2 + mc2s3**2))
    yaw = arctan2(-mc2s3, c2c3)
    pitch = arctan2(s2, c2)
    roll = arctan2(-mc2s1, c1c2)
    return roll, pitch, yaw

def step(dt, now, phase, orientation, seq):
    now += dt
    phase, omega, orientation = integrate(dt, phase, orientation)
    acceleration = accelerometer(orientation)
    _, _, yaw = rpy(orientation)

    header = Header(seq, rospy.Time(now), 'imu_0')
    imu = Imu(header,
              Quaternion(0, 0, 0, 1),
              np.zeros((9,)),
              Vector3(*omega),
              (np.eye(3)*1e-8).reshape((-1,)),
              Vector3(*acceleration),
              (np.eye(3)*1e-8).reshape((-1,)),
             )
    header = Header(seq, rospy.Time(now), 'gyro')
    quat = rotations.quaternion_from_axis_angle(np.r_[0, 0, 1], yaw)
    gyro = Imu(header,
               Quaternion(quat[1], quat[2], quat[3], quat[0]),
               np.diag([0, 0, 1e-10]).reshape((-1,)),
               Vector3(0,0,0),
               np.zeros((9,)),
               Vector3(0,0,0),
               np.zeros((9,)),
              )
    header = Header(seq, rospy.Time(now), 'odom')
    quat = rotations.quaternion_from_rotation(orientation)
    pose = PoseStamped(header,
            Pose(Vector3(0,0,0),
                Quaternion(quat[1], quat[2], quat[3], quat[0])))
    seq += 1
    return (imu, gyro, pose), (now, phase, orientation, seq)

def setiraw(fd):
    save = termios.tcgetattr(0)
    mode = termios.tcgetattr(0)
    mode[tty.IFLAG] = mode[tty.IFLAG] & ~(tty.ICRNL | tty.INPCK | tty.ISTRIP | tty.IXON)
    mode[tty.LFLAG] = mode[tty.LFLAG] & ~(tty.ECHO | tty.ICANON | tty.IEXTEN)
    mode[tty.CC][tty.VMIN] = 0
    mode[tty.CC][tty.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSAFLUSH, mode)
    return save

def run(pause=False):
    """ Run clock and integrator and emit values.
    """
    imupub = rospy.Publisher('~imu', Imu, queue_size=1)
    gyropub = rospy.Publisher('~gyro', Imu, queue_size=1)
    truepose = rospy.Publisher('~pose', PoseStamped, queue_size=1)
    clockpub = rospy.Publisher('/clock', Clock, queue_size=1)

    loop_hz = 125.
    dt = 1./loop_hz

    now = 0.0
    phase = np.zeros((3,))
    orientation = SO3.identity()
    seq = 0
    factor = 1.
    singlestep = False
    state = "PAUSE" if pause else "RUN"

    try:
        saveattrs = setiraw(0)
        while not rospy.is_shutdown():
            if not singlestep:
                time.sleep(dt/factor)
            if not pause:
                (imu, gyro, pose), (now, phase, orientation, seq) = step(dt, now, phase, orientation, seq)
                imupub.publish(imu)
                gyropub.publish(gyro)
                truepose.publish(pose)
                clockpub.publish(Clock(rospy.Time(now)))
                if singlestep:
                    pause = True
                    singlestep = False
            if factor == 1:
                print("\33[2K\r<%s %1.3f>"%(state,now), end="")
            else:
                print("\33[2K\r<RATE %1.3f, %s %1.3f>"%(factor, state, now), end="")
            sys.stdout.flush()
            ready = select.select([0], [], [], 0)
            if len(ready)>0:
                c = os.read(0, 1)
                if c==' ':
                    pause^=True
                    if pause:
                        state="PAUSE"
                    else:
                        state="RUN"
                if pause and c=='s':
                    singlestep = True
                    pause = False
                    state="STEP"
                if c=='+':
                    factor *= 2
                if c=='-':
                    factor /= 2
                if c=='q':
                    print("\33[2K\r<QUIT>", end="")
                    break
    finally:
        termios.tcsetattr(0, termios.TCSANOW, saveattrs)
        print("\33[2K\r<DONE>")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--pause", action="store_true", default=False,
            help="Start paused")
    args, _ = parser.parse_known_args()
    rospy.init_node("SyntheticInertial")
    run(args.pause)
