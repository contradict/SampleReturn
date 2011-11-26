import numpy as np
from numpy import arctan2, sin, cos, pi, sign, sqrt, abs
import pylab

def nrm(v):
    """Vector norm
    """
    return sqrt((v**2).sum(axis=0))

def wheel_angles(phi, kappa, l, w):
    """ Generate wheel angles from robot shape and plane point

    phi   angle to orbit point
    kappa curvature (1/radius) about orbit point. 0 to move
          in a straight line
    l     Distance from rear wheel (origin) to front and center
    w     Distance betwen front wheels

    Returns wheel angles w0, w1, w2

    Wheels are layed out like this:

            (l, w/2) w1
    (0,0)
    w0       ---> Forward (+x)

            (l,-w/2) w2
    """
    w0 = phi-pi/2
    w1 = pi/2 - arctan2( sin(phi) - kappa*w/2, kappa*l - cos(phi))
    w2 = pi/2 - arctan2( sin(phi) + kappa*w/2, kappa*l - cos(phi))
    return w0, w1, w2

def drive(body_pt, body_vel, body_omega, d, l, w):
    """ Move the given body point

    body_pt     Point in body coordinates to move
    body_vel    Velocity for point. can be [0,0] for no linear motion
    body_omega  Rotational rate of point. Can be 0.
                units are radians/s
    d           Wheel Diameter
    l           Length of robot
    w           Width of robot

    Returns wheel angles (radians) and rotational speeds (radians/sec)
    """
    body_speed = nrm(body_vel)
    if abs(body_omega)>0:
        if body_speed>0:
            center_dir = np.r_[-body_vel[1], body_vel[0]]/body_speed
            kappa = body_omega/body_speed
            center_pos = body_pt + center_dir/kappa
        else:
            kappa = 1./nrm(body_pt)
            center_pos = body_pt
        phi = arctan2(center_pos[1], center_pos[0]) + pi/2
        v0 = nrm(center_pos)*body_omega/pi/d
        v1 = nrm(center_pos-[l,  w/2])*body_omega/pi/d
        v2 = nrm(center_pos-[l, -w/2])*body_omega/pi/d
    else:
        kappa = 0
        phi = arctan2(body_vel[1], body_vel[0])+pi/2
        v0 = v1 = v2 = body_speed/pi/d
    w0, w1, w2 = wheel_angles(phi, kappa, l, w)
    return [(w0, v0), (w1, v1), (w2, v2)]

def plot_wheels(phi, kappa, d, l, w):
    w0, w1, w2 = wheel_angles(phi, kappa, l, w)
    fig = pylab.figure(1)
    fig.clf()
    ax = fig.add_subplot(111)
    ax.plot( [-cos(w0)*d/2, cos(w0)*d/2],
             [-sin(w0)*d/2, sin(w0)*d/2],
             label='w0')
    ax.plot( [l-cos(w1)*d/2, l+cos(w1)*d/2],
             [w/2-sin(w1)*d/2, w/2+sin(w1)*d/2],
             label='w1')
    ax.plot( [l-cos(w2)*d/2, l+cos(w2)*d/2],
             [-w/2-sin(w2)*d/2, -w/2+sin(w2)*d/2],
             label='w2')
    if abs(kappa)>0:
        r = 5*sign(kappa)
    else:
        r = 5
    ax.plot( [0, r*cos(w0+pi/2)], [0, r*sin(w0+pi/2)], 'c--')
    ax.plot( [l, l+r*cos(w1+pi/2)], [ w/2, w/2+r*sin(w1+pi/2)], 'c--')
    ax.plot( [l, l+r*cos(w2+pi/2)], [-w/2, -w/2+r*sin(w2+pi/2)], 'c--')
    ax.set_xlim(-0.2, 2.0)
    ax.set_ylim(-1.0, 1.0)
    ax.set_aspect('equal')

