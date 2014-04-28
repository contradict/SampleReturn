import numpy as np
from numpy import pi, tan
import rospy
import platform_motion_msgs.msg as plat_msgs
from copy import deepcopy

def spin():
    path = plat_msgs.Path()
    path.header.stamp = rospy.Time(0)
    path.header.frame_id = 'base_link'

    knot = plat_msgs.Knot()
    knot.header.stamp = rospy.Time(0)
    knot.header.frame_id = 'base_link'

    knot.pose.orientation.w = 1.0;

    N = 5.
    dt = 3.0
    theta0 = 0.01
    omega0 = 0.01
    for x in xrange(int(N)):
        theta = theta0*x/(N-1)
        knot.pose.orientation.w = cos(theta/2.)
        knot.pose.orientation.z = sin(theta/2.)
        knot.twist.angular.z = 0.002 + 0.008*x/(N-1)
        knot.header.stamp += rospy.Duration(dt/N)
        path.knots.append(deepcopy(knot))

    # omega = omega0 + (t<dt/2)?omegaP*2*t/dt:omegaP - (omegaP+omega0)*2*(t-dt/2)/dt
    # theta = theta0 + omega0*t + (t<dt/2)?omegaP*t^2/dt:omegaP*t - (omegaP+omega0)*2*(1/2t^2-t*dt/2)/dt
    # omega0 = 0.01
    # theta0 = 0.01
    # theta(dt) = pi/2 = theta0 + omega0*dt + omegaP*dt - (omegaP+omega0)*2*(1/2dt^2-dt^2/2)/dt
    #                  = theta0 + omega0*dt + omegaP*dt
    # (pi/2-theta0-omega0*dt)/dt = omegaP
    omegaP = (pi/2-theta0-omega0*dt)/dt
    for x in xrange(int(N)):
        s=x/(N-1.)
        t = s*dt
        if s<0.5:
            omega = omega0 + omegaP*2*t/dt
            theta = theta0 + omega0*t + omegaP*t*t
        else:
            omega = omega0 + omegaP - (omegaP+omega0)*2*(t-dt/2.)/dt
            theta = theta0 + omega0*t + omegaP*t-(omegaP+omega0)*2*(t*t/2-t*dt/2)/dt
        knot.pose.orientation.w = cos(theta/2.)
        knot.pose.orientation.z = sin(theta/2.)
        knot.twist.angular.z = omega
        knot.header.stamp += rospy.Duration(dt/N)
        path.knots.append(deepcopy(knot))

    for x in xrange(int(N)):
        s=x/(N-1.)
        t=s*dt
        if s<0.5:
            v = 0.01*t*2/dt
            y = 0.01*t*t/dt
        else:
            v = 0.01-0.01*2*(t-dt/2)/dt
            y = 0.01*t - 0.01*2*(t*t/2.-t*dt/2.)/dt
        knot.pose.position.y = y
        knot.twist.linear.y = v
        knot.header.stamp += rospy.Duration(dt/N)
        path.knots.append(deepcopy(knot))

    return path

def tospin():
    path = plat_msgs.Path()
    path.header.stamp = rospy.Time(0)
    path.header.frame_id = 'base_link'

    knot = plat_msgs.Knot()
    knot.header.stamp = rospy.Time(0)
    knot.header.frame_id = 'base_link'
    knot.pose.orientation.w = 1.0;
    path.knots.append(deepcopy(knot))

    # v=w*r
    R0 = 1e3
    W = 1.13
    phi0 = pi/2-arctan(R0/W)
    omega = 1e-2
    yaw = 0
    phidot = (pi/4)/1.
    # dkappa/dt * wheelbase/(1+(kappa*wheelbase)^2) = dhpi/dt
    dt = pi/2/phidot
    N = 5
    P = np.zeros((2,), dtype='f8')
    V = np.zeros((2,), dtype='f8')
    for x in xrange(N):
        s = float(x)/(N-1)
        t = s*dt
        phi = phi0+s*(pi/2-phi0)
        R = W*tan(pi/2-phi)
        if R>0 :
            omega = min(0.01, 0.01/R)
            v = omega*R
        else:
            omega = 0.01
            v = 0
        yaw += omega*dt/N
        Vprime = v*np.r_[ cos(yaw), sin(yaw) ]
        P += (V*dt/N + Vprime*dt/N)/2.
        V = Vprime
        knot.pose.orientation.w = cos(yaw/2.)
        knot.pose.orientation.z = sin(yaw/2.)
        knot.pose.position.x=P[0]
        knot.pose.position.x=P[1]
        knot.twist.angular.z = omega;
        knot.twist.linear.x = V[0]
        knot.twist.linear.y = V[1]
        knot.header.stamp += rospy.Duration(dt/N)
        knot.header.seq += 1
        path.knots.append(deepcopy(knot))

    omega0 = omega
    yaw0   = yaw
    T = 3.0
    k = 6*(pi/2-yaw0-omega0*T/2.)/T/T/T
    y = -omega0/k/T + T
    for x in xrange(int(N)):
        s = x/float(N-1)
        t = T*s
        omega = omega0 + k*t*(y-t)
        yaw = yaw0 + omega0*t + k*(y*t*t/2. - t*t*t/3.)
        knot.pose.orientation.w = cos(yaw/2.)
        knot.pose.orientation.z = sin(yaw/2.)
        knot.twist.linear.x = 0
        knot.twist.linear.y = 0
        knot.twist.angular.z = omega
        knot.header.stamp += rospy.Duration(T/N)
        knot.header.seq += 1
        path.knots.append(deepcopy(knot))

    # v=w*r
    R0 = 0
    phi0 = pi/2
    omega = 1e-2
    # dkappa/dt * wheelbase/(1+(kappa*wheelbase)^2) = dhpi/dt
    dt = pi/2/phidot
    V = np.zeros((2,), dtype='f8')
    for x in xrange(N):
        s = float(x)/(N-1)
        t = s*dt
        phi = phi0-s*phi0
        R = W*tan(pi/2-phi)
        if R>0 :
            omega = min(0.01, 0.01/R)
            v = omega*R
        else:
            omega = 0.01
            v = 0
        yaw += omega*dt/N
        Vprime = v*np.r_[ cos(yaw), sin(yaw) ]
        P += (V*dt/N + Vprime*dt/N)/2.
        V = Vprime
        knot.pose.orientation.w = cos(yaw/2.)
        knot.pose.orientation.z = sin(yaw/2.)
        knot.pose.position.x=P[0]
        knot.pose.position.x=P[1]
        knot.twist.angular.z = omega;
        knot.twist.linear.x = V[0]
        knot.twist.linear.y = V[1]
        knot.header.stamp += rospy.Duration(dt/N)
        knot.header.seq += 1
        path.knots.append(deepcopy(knot))

    v = 0.01
    dt = 0.2
    V = v*np.r_[ cos(yaw), sin(yaw) ]
    P += V*dt
    knot.pose.position.x=P[0]
    knot.pose.position.x=P[1]
    knot.twist.linear.x = V[0]
    knot.twist.linear.y = V[1]
    knot.twist.angular.z = 0
    knot.header.stamp += rospy.Duration(0.2)
    knot.header.seq += 1

    P += V*dt
    knot.pose.position.x=P[0]
    knot.pose.position.x=P[1]
    knot.header.stamp += rospy.Duration(0.2)
    knot.header.seq += 1
    path.knots.append(deepcopy(knot))

    P += V*dt
    knot.pose.position.x=P[0]
    knot.pose.position.x=P[1]
    knot.twist.linear.x = 0.0
    knot.twist.linear.y = 0.0
    knot.twist.angular.z = 0
    knot.header.stamp += rospy.Duration(0.2)
    knot.header.seq += 1
    path.knots.append(deepcopy(knot))

    knot.header.stamp += rospy.Duration(0.2)
    knot.header.seq += 1
    path.knots.append(deepcopy(knot))

    return path

