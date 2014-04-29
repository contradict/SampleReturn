import numpy as np
from numpy import pi, tan, arctan2, any, isnan, sin, cos, arctan
from numpy.linalg import norm
import rospy
import platform_motion_msgs.msg as plat_msgs
from copy import deepcopy

def yawfromknot(knot):
    return 2*arctan2(knot.pose.orientation.z, knot.pose.orientation.w)

def yawtoknot(knot, yaw):
    knot.pose.orientation.w = cos(yaw/2.)
    knot.pose.orientation.z = sin(yaw/2.)

def scurve(initial, final, dotmax):
    T = abs(final-initial)*3/2./dotmax
    x = lambda s: initial+(final-initial)*3/2.*s*s*(1./2. - s/3.)/0.25
    xdot = lambda s: (final-initial)*3/2.*s*(1.-s)/0.25/T
    return T, x, xdot

def moveR(path, knot, N, R0, R1, phidot=pi/2, omegasmall=3e-3, vsmall=3e-3, wheelbase=1.13):
    # v=w*r
    phi0 = pi/2-arctan(R0/wheelbase)
    phi1 = pi/2-arctan(R1/wheelbase)
    yaw = yawfromknot(knot)
    # dkappa/dt * wheelbase/(1+(kappa*wheelbase)^2) = dhpi/dt
    T, phi, _ = scurve(phi0, phi1, phidot)
    P = np.r_[knot.pose.position.x, knot.pose.position.y]
    V = np.r_[knot.twist.linear.x, knot.twist.linear.y]
    for x in xrange(1,N):
        s = float(x)/(N-1)
        t = s*T
        R = wheelbase*tan(pi/2-phi(s))
        if R>0 :
            omega = min(omegasmall, vsmall/R)
            v = omega*R
        else:
            omega = omegasmall
            v = 0
        yaw += omega*T/N
        Vprime = v*np.r_[ cos(yaw), sin(yaw) ]
        P += (V*T/N + Vprime*T/N)/2.
        V = Vprime
        yawtoknot(knot, yaw)
        knot.pose.position.x=P[0]
        knot.pose.position.y=P[1]
        knot.twist.angular.z = omega;
        knot.twist.linear.x = V[0]
        knot.twist.linear.y = V[1]
        knot.header.stamp += rospy.Duration(T/N)
        knot.header.seq += 1
        path.knots.append(deepcopy(knot))
    return path, knot

def rotate(path, knot, N, yaw1, omegapeak=pi/8):
    yaw0 = yawfromknot(knot)
    omega0 = knot.twist.angular.z
    T, yaw, omega = scurve(yaw0, yaw1, omegapeak)
    P = np.r_[knot.pose.position.x, knot.pose.position.y]
    V = np.r_[knot.twist.linear.x, knot.twist.linear.y]
    for x in xrange(1,int(N)):
        s = x/float(N-1)
        t = T*s
        yawtoknot(knot, yaw(s))
        knot.twist.linear.x = 0
        knot.twist.linear.y = 0
        knot.twist.angular.z = omega(s)
        knot.header.stamp += rospy.Duration(T/N)
        knot.header.seq += 1
        path.knots.append(deepcopy(knot))
    return path, knot

def decel(path, knot, N, a):
    V0 = np.r_[knot.twist.linear.x, knot.twist.linear.y]
    P0 = np.r_[knot.pose.position.x, knot.pose.position.y]
    v0 = norm(V0)
    if v0>0:
        Vdir = V0/v0
    else:
        rospy.logerr("decel from 0")
        return path, knot
    T = v0/a
    v = lambda s : v0 - a*T*s
    x = lambda s : v0*s*T - a*(s*T)*(s*T)/2.
    for n in xrange(1,N):
        s = n/float(N-1)
        t = s*T
        V = Vdir*v(s)
        P = P0 + Vdir*x(s)
        knot.pose.position.x = P[0]
        knot.pose.position.y = P[1]
        knot.twist.linear.x = V[0]
        knot.twist.linear.y = V[1]
        knot.header.seq += 1
        knot.header.stamp += rospy.Duration(T/N)
        path.knots.append(deepcopy(knot))
    return path, knot

def spinLeft():
    path = plat_msgs.Path()
    path.header.stamp = rospy.Time(0)
    path.header.frame_id = 'base_link'

    knot = plat_msgs.Knot()
    knot.header.stamp = rospy.Time(0)
    knot.header.frame_id = 'base_link'
    knot.pose.orientation.w = 1.0;
    path.knots.append(deepcopy(knot))

    # bring radius in from Rlarge to 0
    R0 = 1e5
    R1 = 0
    # each piece is performed in N segments
    N = 20
    path, knot = moveR(path, knot, N, R0, R1)

    # rotate to desiredyaw, not exceeding omegapeak
    desired_yaw = pi/2
    path, knot = rotate(path, knot, N, desired_yaw)

    # return radius to Rlarge
    path, knot = moveR(path, knot, N, R1, R0)

    dt = 0.2

    knot.header.stamp += rospy.Duration(dt)
    knot.header.seq += 1
    path.knots.append(deepcopy(knot))

    return path

def makeHook(path, knot, N, forward=3, R=3, arc0=0, arc1=-pi/2):
    arcdist = R*abs(arc1-arc0)
    straighten = 0.10
    T, l, vl = scurve(0, forward+arcdist+straighten, 1.0)
    yaw0 = yawfromknot(knot)
    P0 = np.r_[knot.pose.position.x, knot.pose.position.y]
    V = np.r_[knot.twist.linear.x, knot.twist.linear.y]
    for n in xrange(1,N):
        s=n/float(N-1)
        t=s*T
        if l(s)<forward:
            # drive forward
            knot.pose.position.x = P0[0] + l(s)*cos(yaw0)
            knot.pose.position.y = P0[1] + l(s)*sin(yaw0)
            P1 = np.r_[knot.pose.position.x, knot.pose.position.y]
            knot.twist.linear.x = vl(s)*cos(yaw0)
            knot.twist.linear.y = vl(s)*sin(yaw0)
        elif l(s)<forward+arcdist:
            # curve around hook
            yaw = yaw0 + arc0 + (l(s)-forward)/arcdist*(arc1-arc0)
            theta = pi/2+yaw
            center = R*np.r_[cos(yaw0-pi/2), sin(yaw0-pi/2)]
            knot.pose.position.x = P1[0] + R*cos(theta) + center[0]
            knot.pose.position.y = P1[1] + R*sin(theta) + center[1]
            P2 = np.r_[knot.pose.position.x, knot.pose.position.y]
            yawtoknot(knot, yaw)
            knot.twist.linear.x = vl(s)*cos(yaw)
            knot.twist.linear.y = vl(s)*sin(yaw)
            knot.twist.angular.z = -vl(s)/R
        else:
            # straighten out
            x=l(s)-forward-arcdist
            yaw = yaw0 + arc1
            theta = pi/2+arc1
            knot.pose.position.x = P2[0] + x*cos(yaw)
            knot.pose.position.y = P2[1] + x*sin(yaw)
            yawtoknot(knot, yaw)
            knot.twist.linear.x = vl(s)*cos(yaw)
            knot.twist.linear.y = vl(s)*sin(yaw)
            knot.twist.angular.z = 0
        knot.header.seq += 1
        knot.header.stamp += rospy.Duration(T/N)
        path.knots.append(deepcopy(knot))
    return path, knot

def hookRight():
    path = plat_msgs.Path()
    path.header.stamp = rospy.Time(0)
    path.header.frame_id = 'base_link'

    knot = plat_msgs.Knot()
    knot.header.stamp = rospy.Time(0)
    knot.header.frame_id = 'base_link'
    knot.pose.orientation.w = 1.0;
    path.knots.append(deepcopy(knot))

    N=20
    path, knot = makeHook(path, knot, N)

    return path

def insertSpinAndHook(stitchedpath):
    hookstart = False
    i=0
    for k in stitchedpath.knots[::-1]:
        smallomega = abs(k.twist.angular.z) < 1e-4
        if hookstart and smallomega:
            break
        elif k.twist.angular.z != 0:
            hookstart = True
        i-=1

    newpath = plat_msgs.Path()
    newpath.header.stamp = rospy.Time.now()
    newpath.header.frame_id = 'odom'

    # for testing to see what shape results after stitching
    #newpath.knots = deepcopy(stitchedpath.knots[:i])
    newpath.knots.append(deepcopy(k))
    rospy.loginfo( "inserting at %f, seq=%d, i=%d", k.header.stamp.to_sec(),
            k.header.seq, i )

    N = 20
    newpath, knot = decel(newpath, k, N, 0.5)
    if not nancheck(newpath):
        rospy.logerr("NaN from decel")

    newpath, knot = moveR(newpath, knot, N, 1e5, 0)
    if not nancheck(newpath):
        rospy.logerr("NaN from moveR 1e5->0")

    yaw = yawfromknot(knot)
    newpath, knot = rotate(newpath, knot, N, yaw+pi/2)
    if not nancheck(newpath):
        rospy.logerr("NaN from rotate")

    newpath, knot = moveR(newpath, knot, N, 0, 1e5)
    if not nancheck(newpath):
        rospy.logerr("NaN from moveR 0->1e5")

    newpath, knot = makeHook(newpath, knot, 20)
    if not nancheck(newpath):
        rospy.logerr("NaN from makeHook")

    return newpath

def nancheck(path):
    for k in path.knots:
        if any([isnan(x) for x in (k.pose.position.x,
                                   k.pose.position.y,
                                   k.pose.position.z,
                                   k.pose.orientation.x,
                                   k.pose.orientation.y,
                                   k.pose.orientation.z,
                                   k.pose.orientation.w,
                                   k.twist.linear.x,
                                   k.twist.linear.y,
                                   k.twist.linear.z,
                                   k.twist.angular.x,
                                   k.twist.angular.y,
                                   k.twist.angular.z)]):
            rospy.logerr("NaN in knot: %s", k)
            return False
    return True

def testPathStitching():

    def sendnext(stitchedpath):
        if sendnext.i>0:
            rospy.loginfo( "insert spin and new hook" )
            if not nancheck(stitchedpath):
                rospy.logerr("NaN in stitched")
                sendnext.cleanup()
            newpath = insertSpinAndHook(stitchedpath)
            if nancheck(newpath):
                sendnext.planpub.publish(newpath)
            else:
                sendnext.cleanup()
        sendnext.i -= 1

    sendnext.i = 4

    sendnext.planpub = rospy.Publisher("/motion/planned_path", plat_msgs.Path)

    sendnext.stitchsub = rospy.Subscriber("/motion/stitched_path", plat_msgs.Path,
            sendnext)

    def cleanup(sendnext=sendnext):
        rospy.loginfo( "Clean up" )
        sendnext.stitchsub.unregister()
        sendnext.planpub.unregister()
    sendnext.cleanup = cleanup

    while (sendnext.planpub.get_num_connections() < 1 or
           sendnext.stitchsub.get_num_connections()<1):
        rospy.sleep(rospy.Duration(1.0))

    rospy.loginfo("Start with hook")
    h = hookRight()
    sendnext.planpub.publish(hookRight())

    return cleanup

# Left commented out for experimentation in ipython
# turn it back on to run from command line
# if __name__ == "__main__":
#     rospy.init_node("stitchtest")
#     cleanup = testPathStitching()
#     rospy.on_shutdown(cleanup)
#     rospy.spin()

