import numpy as np
from numpy import linspace, cos, sin
from scipy.optimize import fsolve

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
    L = sqrt(dx*dx+dy*dy)
    b = 12*dyaw/L/3.
    c = dyaw/L/12.
    x0 = np.r_[L, b, c]
    xopt, infodict, ier, mesg = fsolve(error, x0, full_output=True)
    print "F(xopt)=%s"%infodict['fvec']
    if ier != 1:
        raise Exception( "Did not converge (%d): %s"%(ier, mesg) )
    return xopt

# compute polynomial parameters for path
xopt = optimize(0.3, 0.2, pi/4, initial_yaw=-pi/8)
# integrate points
p = integrate_pose_cubic(xopt[0], xopt[1], xopt[2], initial_yaw=-pi/8)
# plot the answer
plot(p[:,0], p[:,1])

