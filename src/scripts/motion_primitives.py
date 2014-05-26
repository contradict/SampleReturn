import numpy as np
import math
from numpy import pi, tan, arctan2, any, isnan, sin, cos, arctan, linspace
from numpy.linalg import norm
from scipy.optimize import fsolve
import rospy
import platform_motion_msgs.msg as plat_msgs
import matplotlib.pyplot as plt
from copy import deepcopy

def yawfromknot(knot):
    return 2*arctan2(knot.pose.orientation.z, knot.pose.orientation.w)

def yawtoknot(knot, yaw):
    knot.pose.orientation.w = cos(yaw/2.0)
    knot.pose.orientation.z = sin(yaw/2.0)

def integrate_pose_quadratic(L, dyaw, N=100):
    pose=np.r_[[[0,0,0]]]
    dl=L/N
    b=6*dyaw/L
    for s in linspace(0,1,N):
        x,y,yaw = pose[-1,:]
        kappa = b*s-b*s*s
        x += dl*cos(yaw)
        y += dl*sin(yaw)
        yaw += kappa*L/N
        pose = np.r_[pose, [[x,y,yaw]]]
    return pose

def integrate_pose_cubic(L, b, c, initial_yaw=0, N=100):
    pose=np.r_[[[0,0,initial_yaw]]]
    dl=L/N
    d=-b-c
    for s in linspace(0,1,N):
        x,y,yaw = pose[-1]
        kappa = b*s + c*s*s +d*s*s*s
        x += dl*cos(yaw)
        y += dl*sin(yaw)
        yaw += kappa*dl
        pose = np.r_[pose, [[x,y,yaw]]]
    return pose

def optimize(dx, dy, dyaw, initial_yaw=0):
    error = lambda x : integrate_pose_cubic(x[0], x[1], x[2], initial_yaw=initial_yaw)[-1]-np.r_[dx,dy,dyaw+initial_yaw]
    L = math.sqrt(dx*dx+dy*dy)
    b = 12*dyaw/L/3.
    c = dyaw/L/12.
    x0 = np.r_[L, b, c]
    xopt, infodict, ier, mesg = fsolve(error, x0, full_output=True)
    #print "F(xopt)=%s"%infodict['fvec']
    if ier != 1:
        raise Exception( "Did not converge (%d): %s"%(ier, mesg) )
    return xopt

def scurve(initial, final, dotmax):
    #T = abs(final-initial)*3/2.0/dotmax
    T = abs(final-initial)/dotmax
    #x = lambda s: initial+(final-initial)*3/2.0*s*s*(1.0/2.0 - s/3.0)*4.0
    x = lambda s: initial*(1.0-s) + final*(s)
    #xdot = lambda s: (final-initial)*3/2.0*s*(1.0-s)*4.0/T
    xdot = lambda s: (final-initial)/T
    return T, x, xdot

def extendPathGridAlign(path, knot, gridspacing, extendnumber=3):
    yaw = yawfromknot(path.knots[-1])
    P0 = np.r_[path.knots[-1].pose.position.x, path.knots[-1].pose.position.y]
    extenddist = extendnumber*gridspacing
    P1 = np.r_[round((P0[0] + extenddist*cos(yaw))/gridspacing)*gridspacing, round((P0[1] + extenddist*sin(yaw))/gridspacing)*gridspacing]
    yawtoknot(knot, yaw)

    for i in xrange(1, extendnumber+1):
        lerp = float(i)/extendnumber
        knot.pose.position.x = (1.0-lerp)*P0[0] + lerp*P1[0]
        knot.pose.position.y = (1.0-lerp)*P0[1] + lerp*P1[1]
        knot.twist.linear.x = 0
        knot.twist.linear.y = 0
        knot.twist.angular.z = 0;
        path.knots.append(deepcopy(knot))
    return path, knot

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
    omega = knot.twist.angular.z
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

def makeForward(path, knot, forward, yaw, gridspacing=0.1, pathspacing=0.1):
    # special case clamp forward to be aligned to gridspacing
    # for some angles
    epsilon = 0.0001
    if math.fmod(yaw, pi/2.0) < epsilon:
        forward = round(forward/gridspacing)*gridspacing
    elif math.fmod(yaw, pi/4.0) < epsilon:
        deg45gridspacing = gridspacing * math.sqrt(2)
        forward = round(forward/deg45gridspacing)*deg45gridspacing
    elif math.fmod(yaw, pi/8.0) < epsilon:
        deg22p5gridspacing = gridspacing * math.sqrt(5.0*5.0 + 12.0*12.0)
        forward = round(forward/deg22p5gridspacing)*deg22p5gridspacing

    # make forward path
    T, l, vl = scurve(0, forward, 1.0)
    P0 = np.r_[knot.pose.position.x, knot.pose.position.y]
    forwardN = math.ceil(forward / pathspacing)
    forwardspacing = forward / forwardN
    forwardcount = 0
    while forwardcount < (forwardN - epsilon):
        forwardtraveled = forwardcount * forwardspacing
        s = forwardtraveled / forward 
        knot.pose.position.x = P0[0] + l(s)*cos(yaw)
        knot.pose.position.y = P0[1] + l(s)*sin(yaw)
        yawtoknot(knot, yaw)
        knot.twist.linear.x = vl(s)*cos(yaw)
        knot.twist.linear.y = vl(s)*sin(yaw)

        knot.header.seq += 1
        knot.header.stamp += rospy.Duration(T/forwardN)
        path.knots.append(deepcopy(knot))
        forwardcount = forwardcount + 1

    # copy the last point in exactly
    knot.pose.position.x = P0[0] + l(1.0)*cos(yaw)
    knot.pose.position.y = P0[1] + l(1.0)*sin(yaw)
    yawtoknot(knot, yaw)
    knot.twist.linear.x = vl(1.0)*cos(yaw)
    knot.twist.linear.y = vl(1.0)*sin(yaw)

    knot.header.seq += 1
    knot.header.stamp += rospy.Duration(T/forwardN)
    path.knots.append(deepcopy(knot))

    return path, knot

def makeHook(path, knot, forward=3, R=3, initialyaw=0, deltayaw=-pi/2, gridspacing=0.1, pathspacing=0.1):
    # find the end point, with grid alignment
    straighten = 0.5
    arcdist = R*abs(deltayaw)
    yaw0 = initialyaw
    P0 = np.r_[knot.pose.position.x, knot.pose.position.y]
    P1 = np.r_[P0[0] + forward*cos(yaw0), P0[1] + forward*sin(yaw0)]
    yaw = yaw0 + deltayaw
    sign = np.sign(deltayaw)
    theta = sign * -pi/2+yaw
    center = R*np.r_[cos(yaw0+sign*pi/2), sin(yaw0+sign*pi/2)] + P1
    P2 = np.r_[R*cos(theta) + center[0], R*sin(theta) + center[1]]
    P3 = np.r_[P2[0] + straighten*cos(yaw), P2[1] + straighten*sin(yaw)]
    P4 = np.r_[round(P3[0]/gridspacing)*gridspacing, round(P3[1]/gridspacing)*gridspacing]
    # move backward from P4 along the right yaw and find a 
    #new R to bridge the two straight paths
    straightenvec = np.r_[cos(yaw), sin(yaw)]
    determinant = -straightenvec[0]*(cos(yaw)-cos(yaw0)) + straightenvec[1]*(sin(yaw0)-sin(yaw))
    straightendist = (1.0/determinant)*((cos(yaw)-cos(yaw0))*(P1[0]-P4[0]) + (sin(yaw)-sin(yaw0))*(P1[1]-P4[1]))
    R = sign*(1.0/determinant)*(straightenvec[1]*(P1[0]-P4[0]) - straightenvec[0]*(P1[1]-P4[1]))
    straighten = straightendist
    P5 = P4 - straightendist*straightenvec
    arcdist = R*abs(deltayaw)
    
    # compute the path
    totaldistance = forward+arcdist+straighten
    T, l, vl = scurve(0, totaldistance, 1.0)

    # drive forward
    epsilon = 0.0001
    forwardN = math.ceil(forward / pathspacing)
    forwardspacing = forward / forwardN
    forwardcount = 0
    while forwardcount <= (forwardN - epsilon):
        forwardtraveled = forwardcount * forwardspacing
        s = forwardtraveled / totaldistance
        knot.pose.position.x = P0[0] + forwardtraveled*cos(yaw0)
        knot.pose.position.y = P0[1] + forwardtraveled*sin(yaw0)
        yawtoknot(knot, yaw0)
        knot.twist.linear.x = vl(s)*cos(yaw0)
        knot.twist.linear.y = vl(s)*sin(yaw0)
        knot.header.seq += 1
        knot.header.stamp += rospy.Duration(T/forwardN)
        path.knots.append(deepcopy(knot))
        forwardcount = forwardcount + 1

    # curve around hook
    curveN = math.ceil(arcdist / pathspacing)
    curvespacing = arcdist / curveN
    curvecount = 0
    while curvecount < (curveN - epsilon):
        curvetraveled = curvecount * curvespacing
        s = (curvetraveled + forward) / totaldistance
        yaw = yaw0 + curvetraveled/arcdist*deltayaw
        sign = np.sign(deltayaw)
        theta = sign * -pi/2+yaw
        center = R*np.r_[cos(yaw0+sign*pi/2), sin(yaw0+sign*pi/2)]
        knot.pose.position.x = P1[0] + R*cos(theta) + center[0]
        knot.pose.position.y = P1[1] + R*sin(theta) + center[1]
        yawtoknot(knot, yaw)
        knot.twist.linear.x = vl(s)*cos(yaw)
        knot.twist.linear.y = vl(s)*sin(yaw)
        knot.twist.angular.z = -vl(s)/R
        knot.header.seq += 1
        knot.header.stamp += rospy.Duration(T/curveN)
        path.knots.append(deepcopy(knot))
        curvecount = curvecount + 1

    # straighten out
    straightN = math.ceil(straighten / pathspacing)
    straightspacing = straighten / straightN
    straightcount = 0
    while straightcount < (straightN - epsilon): # save the last point for later
        straighttraveled = straightcount * straightspacing
        s = (straighttraveled + forward + arcdist) / totaldistance
        yaw = yaw0 + deltayaw
        knot.pose.position.x = P5[0] + straighttraveled*cos(yaw)
        knot.pose.position.y = P5[1] + straighttraveled*sin(yaw)
        yawtoknot(knot, yaw)
        knot.twist.linear.x = vl(s)*cos(yaw)
        knot.twist.linear.y = vl(s)*sin(yaw)
        knot.twist.angular.z = 0
        knot.header.seq += 1
        knot.header.stamp += rospy.Duration(T/straightN)
        path.knots.append(deepcopy(knot))
        straightcount = straightcount + 1

    # insert the exact end point
    yaw = initialyaw + deltayaw
    knot.pose.position.x = P4[0]
    knot.pose.position.y = P4[1]
    yawtoknot(knot, yaw)
    knot.twist.linear.x = vl(1.0)*cos(yaw)
    knot.twist.linear.y = vl(1.0)*sin(yaw)
    knot.twist.angular.z = 0
    path.knots.append(deepcopy(knot))
    return path, knot

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

def smoothhook(arcdist, initialyaw, deltayaw, gridSpacing=0.1, pathSpacing=0.1):
    path = plat_msgs.Path()
    knot = plat_msgs.Knot()
    dx = arcdist*cos(initialyaw+deltayaw) # approximate dx, dy from arcdist
    dy = arcdist*sin(initialyaw+deltayaw)
    dx = round(dx/gridSpacing)*gridSpacing
    dy = round(dy/gridSpacing)*gridSpacing # clamp dx, dy to the grid
    xopt = optimize(dx, dy, deltayaw, initialyaw)
    poses = integrate_pose_cubic(xopt[0], xopt[1], xopt[2], initialyaw)
    N = int(len(poses)*pathSpacing / arcdist)
    for pose in poses[0::N]:
        yawtoknot(knot, pose[2])
        knot.pose.position.x = pose[0]
        knot.pose.position.y = pose[1]
        knot.twist.linear.x = 0
        knot.twist.linear.y = 0
        knot.twist.angular.z = 0 # Don't bother with omega_dot
        path.knots.append(deepcopy(knot))

    last_x = path.knots[-1].pose.position.x
    last_y = path.knots[-1].pose.position.y
    if ((last_x-dx)*(last_x-dx) + (last_y-dy)*(last_y-dy)) < pathSpacing*pathSpacing:
        # Replace the last knot's pose with the exact pose
        path.knots[-1].pose.position.x = dx
        path.knots[-1].pose.position.y = dy
    else:
        # Append the last knot's pose with the exact pose
        yawtoknot(knot, initialyaw+deltayaw)
        knot.pose.position.x = dx
        knot.pose.position.y = dy
        knot.twist.linear.x = 0
        knot.twist.linear.y = 0
        knot.twist.angular.z = 0 # Don't bother with omega_dot
        path.knots.append(deepcopy(knot))

    return path

def hook(forward, R, initialyaw, deltayaw, gridSpacing=0.1, pathSpacing=0.1):
    path = plat_msgs.Path()
    knot = plat_msgs.Knot()
    yawtoknot(knot, initialyaw)

    path, knot = makeHook(path, knot, forward, R, initialyaw, deltayaw, gridSpacing, pathSpacing)

    return path

def forward(forward, yaw, gridSpacing=0.1, pathSpacing=0.1):
    path = plat_msgs.Path()
    knot = plat_msgs.Knot()

    path, knot = makeForward(path, knot, forward, yaw, gridSpacing, pathSpacing)

    return path

def turnInPlace(initialyaw, deltayaw):
    path = plat_msgs.Path()
    knot = plat_msgs.Knot()
    yawtoknot(knot, initialyaw)
    
    N = 20
    #path, knot = decel(path, knot, N, 0.5)
    #if not nancheck(path):
    #   rospy.logerr("NaN from decel")

    path, knot = moveR(path, knot, N, 1e5, 0)
    if not nancheck(path):
        rospy.logerr("NaN from moveR 1e5->0")

    yaw = yawfromknot(knot)
    path, knot = rotate(path, knot, N, yaw+deltayaw)
    if not nancheck(path):
        rospy.logerr("NaN from rotate")

    path, knot = moveR(path, knot, N, 0, 1e5)
    if not nancheck(path):
        rospy.logerr("NaN from moveR 0->1e5")

    return path

def fakeTurnInPlace(initialyaw, deltayaw):
    path = plat_msgs.Path()
    knot = plat_msgs.Knot()
    yawtoknot(knot, initialyaw)
    path.knots.append(deepcopy(knot))
    yawtoknot(knot, initialyaw + deltayaw)
    path.knots.append(deepcopy(knot))
    
    return path

def plotPath(path):
    max_distance = 4.0
    for knot in path.knots:
        plt.plot(knot.pose.position.x, knot.pose.position.y, marker=(3, 0, -90 + 180/pi*yawfromknot(knot)))
    plt.xlim([-max_distance, max_distance])
    plt.ylim([-max_distance, max_distance])
    plt.show()

def openPrimitivesFile(filename, gridspacing):
    primfile = open(filename, 'w+')
    primdata = {'file' : primfile, 'angles' : 0, 'totalprims' : 0, 'gridspacing' : gridspacing}
    primfile.write("resolution_m: %.6f\n" % 0)
    primfile.write("numberofangles: %2d\n" % 0)
    primfile.write("totalnumberofprimitives: %3d\n" % 0)
    return primdata

def addToPrimitivesFile(primdata, pathdata):
    primfile = primdata['file']
    gridspacing = primdata['gridspacing']
    primitiveid = 0
    for pathdatum in pathdata:
        path = pathdatum['path']
        cost = pathdatum['cost']
        initialpos = path.knots[-1].pose.position
        primfile.write("primID: %d\n" % primitiveid)
        primfile.write("startangle_c: %d\n" % primdata['angles'])
        primfile.write("endpose_c: %d %d %d\n" % (round(initialpos.x/gridspacing), round(initialpos.y/gridspacing), pathdatum['endpose_c']))
        primfile.write("additionalactioncostmult: %d\n" % cost)
        primfile.write("intermediateposes: %d\n" % len(path.knots))
        for knot in path.knots:
            yaw = yawfromknot(knot)
            primfile.write("%.4f %.4f %.4f\n" % (knot.pose.position.x, knot.pose.position.y, yaw))
        primitiveid = primitiveid + 1

    primdata['totalprims'] = primdata['totalprims'] + len(pathdata)
    primdata['angles'] = primdata['angles'] + 1

def closePrimitivesFile(primdata):
    primfile = primdata['file']
    primfile.seek(0, 0)
    primfile.write("resolution_m: %.6f\n" % primdata['gridspacing'])
    primfile.write("numberofangles: %2d\n" % primdata['angles'])
    primfile.write("totalnumberofprimitives: %3d\n" % primdata['totalprims'])
    primfile.close()

def generateMotionPrimitives(showplots=False):
    gridspacing = 0.1
    pathspacing = 0.2
    numangles = 8
    prims = [[0.5, 2.3], [0.25, 1.2]] # list of [forward, radius]
    primfile = openPrimitivesFile("motion_primitives.mprim", gridspacing)
    for i in xrange(0,numangles):
        initialyaw = 2*pi*i/numangles
        deltayaw = 2*pi/numangles
        pathdata = []
        for prim in prims:
            forwarddist = prim[0] + prim[1]*abs(deltayaw) # Make the forward dist as long as the hook paths

            # Forward and right turn
            #path = smoothhook(forwarddist, initialyaw, -deltayaw, gridspacing, pathspacing)
            #pathdata.append({'path' : path, 'cost' : 2, 'endpose_c' : i-1})
            #if showplots:
            #    plotPath(path)

            # Forward
            path = forward(forwarddist, initialyaw, gridspacing, pathspacing)
            pathdata.append({'path' : path, 'cost' : 1, 'endpose_c' : i})
            if showplots:
                plotPath(path)

            # Forward and left turn
            #path = smoothhook(forwarddist, initialyaw, deltayaw, gridspacing, pathspacing)
            #pathdata.append({'path' : path, 'cost' : 2, 'endpose_c' : i+1})
            #if showplots:
            #    plotPath(path)

        # Short forward
        forwarddist = gridspacing
        path = forward(forwarddist, initialyaw, gridspacing, pathspacing)
        pathdata.append({'path' : path, 'cost' : 1, 'endpose_c' : i})
        if showplots:
            plotPath(path)

        # Turn in place to the right
        #path = turnInPlace(initialyaw, -deltayaw)
        #pathdata.append({'path' : path, 'cost' : 3, 'endpose_c' : i-1})
        #if showplots:
        #   plotPath(path)

        # Turn in place to the left 
        #path = turnInPlace(initialyaw, deltayaw)
        #pathdata.append({'path' : path, 'cost' : 3, 'endpose_c' : i+1})
        #if showplots:
        #   plotPath(path)

        # Turn in place to the right
        path = fakeTurnInPlace(initialyaw, -deltayaw)
        pathdata.append({'path' : path, 'cost' : 3, 'endpose_c' : i-1})
        if showplots:
            plotPath(path)

        # Turn in place to the left 
        path = fakeTurnInPlace(initialyaw, deltayaw)
        pathdata.append({'path' : path, 'cost' : 3, 'endpose_c' : i+1})
        if showplots:
            plotPath(path)

        addToPrimitivesFile(primfile, pathdata)

    closePrimitivesFile(primfile)
    

if __name__ == "__main__":
    generateMotionPrimitives(False)
