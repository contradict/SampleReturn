import numpy as np
from numpy import arctan2, sin, cos, sqrt, pi, diff, allclose, abs
import pylab

def makePath(N=100, A=3, omega=0.2):
    t=linspace(0,2*pi/omega,N)
    x=2*A*sin(omega*t)
    xdot=2*A*omega*cos(omega*t)
    xddot=-2*A*omega*omega*sin(omega*t)
    y=A*sin(2*omega*t)
    ydot=2*A*omega*cos(2*omega*t)
    yddot=-4*A*omega*omega*sin(2*omega*t)
    #theta=arctan2(ydot, xdot)
    #theta = 2*arctan((sqrt(xdot**2+ydot**2)-xdot)/ydot)
    f = lambda x : 2.*arctan(x)
    fprime = lambda x : 2./(1.+x**2)
    g = lambda x,y : (sqrt(x**2 + y**2)-x)/y
    gprime = lambda x,y,xdot,ydot : ((x**2+y**2)**(-1./2.)*(x*xdot+y*ydot)-xdot)/y - (sqrt(x**2+y**2)-x)/y**2*ydot
    theta = f(g(xdot, ydot))
    thetadot=fprime(g(xdot,ydot))*gprime(xdot, ydot, xddot, yddot)
    return np.c_[x,y,theta,xdot,ydot,thetadot,t]

def drawPath(p,N=1):
    base=p[::N,:2]
    heading=np.c_[cos(p[::N,2]), sin(p[::N,2])]
    velocity=p[::N,3:5]
    f=pylab.figure(1)
    f.clear()
    ax1 = f.add_subplot(221)
    ax1.set_title("heading")
    ax1.set_aspect(1)
    ax1.quiver(base[:,0], base[:,1], heading[:,0], heading[:,1])
    ax2 = f.add_subplot(222)
    ax2.quiver(base[:,0], base[:,1], velocity[:,0], velocity[:,1])
    ax2.set_title("velocity")
    ax2.set_aspect(1)
    tdot = p[::N,5:6]*np.c_[cos(p[::N,2]+pi/2), sin(p[::N,2]+pi/2)]
    ax3=f.add_subplot(223)
    ax3.set_title("thetadot")
    ax3.set_aspect(1)
    ax3.quiver(base[:,0], base[:,1], heading[:,0], heading[:,1])
    s=3.3
    ax3.quiver(base[:,0]+heading[:,0]/s, base[:,1]+heading[:,1]/s, tdot[:,0], tdot[:,1])
    ax4=f.add_subplot(224)
    ax4.set_title("theta")
    ax4.plot(p[:,6], p[:,2])
    f.canvas.draw()

def unwrap(theta):
    theta = np.where(theta<-pi, theta+2*pi, theta)
    theta = np.where(theta>pi, theta-2*pi, theta)
    return theta

def path2local(p, R, small=1e-3):
    x = p[:,0:1]
    y = p[:,1:2]
    theta = p[:,2:3]
    xdot = p[:,3:4]
    ydot = p[:,4:5]
    thetadot = p[:,5:6]
    t=p[:,6:7]
    ct = cos(theta)[...,None]
    st = sin(theta)[...,None]
    rot = np.append(np.c_[ct, st], np.c_[-st, ct], axis=1)
    dtheta = unwrap(np.r_[diff(theta, axis=0), [[0]]])
    ct = cos(dtheta)[...,None]
    st = sin(dtheta)[...,None]
    drot = np.append(np.c_[1-ct, st], np.c_[-st, 1-ct], axis=1)
    rdx=np.einsum("...ij,...jk,...kl", drot, rot, R.reshape((1,2,1)))
    # appending zero here is equivalent to assuming all paths end stopped
    dx = np.r_[diff(x, axis=0), [[0]]]+rdx[:,0]
    dy = np.r_[diff(y, axis=0), [[0]]]+rdx[:,1]
    # V = (thetadot Zhat) x R + v
    V = np.c_[-thetadot*R[1], thetadot*R[0]] + np.c_[xdot, ydot]
    Vx = V[:,0:1]
    Vy = V[:,1:2]
    # ksidot = ||V||
    ksidot = sqrt(Vx**2+Vy**2)
    # phi = arg(V)-theta
    phi = unwrap(arctan2(Vy, Vx) - theta)
    dksi = np.where(abs(dtheta)<small, sqrt(dx**2+dy**2),
                                  abs(dtheta)*sqrt(dx**2+dy**2)/sqrt(2*(1.-cos(dtheta))))
    # V = ksidot exp(i(phi+theta))
    alpha = unwrap(phi+theta)
    # Vdot = ksiddot exp(i alpha) + i alpha (phidot+thetadot) ksidot exp(i alpha)
    dt = np.diff(t, axis=0)
    Vdot = np.r_[diff(V, axis=0)/dt, [[0,0]]]
    Vdotx = Vdot[:,0:1]
    Vdoty = Vdot[:,1:2]
    ksiddot = (Vx**2+Vy**2)**(-1./2)*(Vx*Vdotx + Vy*Vdoty)
    # Vdot = (ksiddot + i (phidot+thetadot) ksidot) exp(i alpha)
    # (Vdot exp(-i alpha) - ksiddot)/(i ksidot) - thetadot = phidot
    # (Vdot exp(-i (alpha+pi/2)) + i ksiddot)/(ksidot) - thetadot = phidot
    # ([Vdotx cos(-alpha-pi/2) - Vdoty sin(-alpha-pi/2),
    #   Vdoty cos(-alpha-pi/2) + Vdotx sin(-alpha-pi/2)+ksiddot])/(ksidot) - thetadot = phidot
    phidot = (Vdotx*cos(-alpha-pi/2) - Vdoty*sin(-alpha-pi/2))/ksidot - thetadot
    # this should be very small (analytically, it should be zero) everywhere
    imphidot = (Vdoty*cos(-alpha-pi/2) + Vdotx*sin(-alpha-pi/2)+ksiddot)/ksidot
    assert(allclose(imphidot, 0))
    return np.c_[phi, phidot, dksi, ksidot, t], alpha

def plotlocal(l):
    f=pylab.figure(2)
    f.clear()
    ax4=f.add_subplot(414)
    ax1=f.add_subplot(411, sharex=ax4)
    pylab.setp( ax1.get_xticklabels(), visible=False)
    ax1.set_title("phi")
    ax1.plot(l[:,4], l[:,0])
    ax2=f.add_subplot(412, sharex=ax4)
    pylab.setp( ax2.get_xticklabels(), visible=False)
    ax2.set_title("phidot")
    ax2.plot(l[:,4], l[:,1])
    ax3=f.add_subplot(413, sharex=ax4)
    pylab.setp( ax3.get_xticklabels(), visible=False)
    ax3.set_title("dksi")
    ax3.plot(l[:,4], l[:,2])
    ax4.set_title("ksidot")
    ax4.plot(l[:,4], l[:,3])
    f.canvas.draw()

