import numpy as np
from numpy import arctan2, sin, cos, sqrt, pi, diff, allclose, abs
import pylab

def makeCircle(R=5.0, N=100, a=0.1, vmax=1.0):
    Taccel = vmax/a
    Saccel = 0.5*a*Taccel*Taccel
    Tconst = (2*pi*R-2*Saccel)/vmax
    if Tconst<0:
        Taccel = sqrt(2*pi*R/a)
        Tconst = 0
    T=2*Taccel+Tconst
    t=linspace(0,T,N)[:,None]
    @np.vectorize
    def velocity(t):
        if t<Taccel:
            return a*t
        elif t>Taccel+Tconst:
            return vmax-a*(t-Taccel-Tconst)
        else:
            return vmax
    @np.vectorize
    def distance(t):
        if(t<Taccel):
            return 0.5*a*t*t
        elif(t<Taccel+Tconst):
            return 0.5*a*Taccel*Taccel + vmax*(t-Taccel)
        else:
            tau=(t-Taccel-Tconst)
            return 0.5*a*Taccel*Taccel + vmax*Tconst + vmax*tau - 0.5*a*tau*tau
    v=velocity(t)
    s=distance(t)
    theta=s/R
    omega=v/R
    x=R*sin(theta)
    y=R*(1-cos(theta))
    xdot=v*cos(theta)
    ydot=v*sin(theta)
    return np.c_[x, y, theta, xdot, ydot, omega, t]


def makeLine(N=100, a=0.1, vmax=1.0, Tconst=2.0):
    Taccel = vmax/a
    T=2*Taccel+Tconst
    t=linspace(0,T,N)[:,None]
    @np.vectorize
    def velocity(t):
        if t<Taccel:
            return a*t
        elif t>Taccel+Tconst:
            return vmax-a*(t-Taccel-Tconst)
        else:
            return vmax
    @np.vectorize
    def distance(t):
        if(t<Taccel):
            return 0.5*a*t*t
        elif(t<Taccel+Tconst):
            return 0.5*a*Taccel*Taccel + vmax*(t-Taccel)
        else:
            tau=(t-Taccel-Tconst)
            return 0.5*a*Taccel*Taccel + vmax*Tconst + vmax*tau - 0.5*a*tau*tau

    z=np.zeros((N,1))
    return np.c_[distance(t), z, z, velocity(t), z, z, t]

def makeFigureEight(N=100, A=3, omega=0.2, a=0.1):
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
    path=np.c_[x,y,theta,xdot,ydot,thetadot,t]
    v0=sqrt(xdot[0]**2 + ydot[0]**2)
    d0=1./2.*v0**2/a
    P0 = path[0:1,0:2]+[[d0*cos(theta[0]+pi), d0*sin(theta[0]+pi)]]
    t0=-v0/a
    path = np.r_[np.c_[ P0, theta[0], 0, 0, 0, t0], path]
    ve=sqrt(xdot[-1]**2 + ydot[-1]**2)
    de=1./2.*ve**2/a
    Pe = path[-1:,0:2]+[[de*cos(theta[-1]), de*sin(theta[-1])]]
    te = t[-1]+ve/a
    path = np.r_[path, np.c_[ Pe, theta[-1], 0, 0, 0, te]]
    path[:,-1] -= path[0,-1]
    return path

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
    ax1.set_xlim(base[:,0].min()-0.25, base[:,0].max()+0.25)
    ax1.set_ylim(base[:,1].min()-0.25, base[:,1].max()+0.25)

    ax2 = f.add_subplot(222)
    ax2.set_title("velocity")
    ax2.set_aspect(1)
    ax2.quiver(base[:,0], base[:,1], velocity[:,0], velocity[:,1])
    ax2.set_xlim(base[:,0].min()-0.25, base[:,0].max()+0.25)
    ax2.set_ylim(base[:,1].min()-0.25, base[:,1].max()+0.25)

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

def path2local(p, R, initialphi=0, small=1e-3):
    x = p[:,0:1]
    y = p[:,1:2]
    theta = p[:,2:3]
    xdot = p[:,3:4]
    ydot = p[:,4:5]
    thetadot = p[:,5:6]
    t=p[:,6:7]
    ct = cos(-theta)[...,None]
    st = sin(-theta)[...,None]
    rot = np.append(np.c_[ct, st], np.c_[-st, ct], axis=1)
    dtheta = unwrap(np.r_[diff(theta, axis=0), [[0]]])
    ct = cos(dtheta)[...,None]
    st = sin(dtheta)[...,None]
    drot = np.append(np.c_[ct-1, st], np.c_[-st, ct-1], axis=1)
    rdx=np.einsum("...ij,...jk,...kl", drot, rot, R.reshape((1,2,1)))
    # appending zero here is equivalent to assuming all paths end stopped
    dx = np.r_[diff(x, axis=0), [[0]]]+rdx[:,0]
    dy = np.r_[diff(y, axis=0), [[0]]]+rdx[:,1]
    # V = (thetadot Zhat) x R + v
    td = thetadot[...,None]
    z = np.zeros_like(td)
    omegacross = np.append(np.c_[z, -td], np.c_[td, z], axis=1)
    V = np.einsum("...ij,...jk,...kl", omegacross, rot, R.reshape((1,2,1)))[...,0] + np.c_[xdot, ydot]
    #V = np.c_[-thetadot*R[1], thetadot*R[0]] + np.c_[xdot, ydot]
    Vx = V[:,0:1]
    Vy = V[:,1:2]
    # ksidot = ||V||
    ksidot = sqrt(Vx**2+Vy**2)
    # phi = arg(V)-theta
    phi = unwrap(arctan2(Vy, Vx) - theta)
    phi = np.where( abs(abs(phi)-pi)<0.001, pi, phi )
    last_good_phi=initialphi
    for i in xrange(phi.shape[0]):
        if ksidot[i]<small:
            phi[i]=last_good_phi
        else:
            last_good_phi = phi[i]
    dksi = np.where(abs(dtheta)<small, sqrt(dx**2+dy**2),
                                  abs(dtheta)*sqrt(dx**2+dy**2)/sqrt(2*(1.-cos(dtheta))))
    if 0:
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
        phidot = np.where(ksidot>small, (Vdotx*cos(-alpha-pi/2) -
            Vdoty*sin(-alpha-pi/2))/ksidot - thetadot, 0)
        # this should be very small (analytically, it should be zero) everywhere
        imphidot = np.where(ksidot>small, (Vdoty*cos(-alpha-pi/2) +
            Vdotx*sin(-alpha-pi/2)+ksiddot)/ksidot, 0)
        assert(allclose(imphidot, 0))
    else:
        for i in xrange(phi.shape[0]):
            if abs(phi[i]-pi/2) < 0.001:
                phi[i] = pi/2
            elif abs(phi[i]+pi/2) < 0.001:
                phi[i] = -pi/2
            elif abs(phi[i])>pi/2:
                phi[i] = unwrap(phi[i]+pi)
                ksidot[i] *= -1
                dksi[i] *= -1
        phidot = np.r_[ np.zeros((1,1)), 
                        ((phi[1:-1,:]-phi[:-2,:])/(t[1:-1,:]-t[:-2,:]) + 
                         (phi[2:,:]-phi[1:-1,:])/(t[2:,:]-t[1:-1,:]))/2.,
                        np.zeros((1,1)) ]
    return np.c_[phi, phidot, dksi, ksidot, t]

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

def rot(theta, v):
    ct = cos(theta)[...,None]
    st = sin(theta)[...,None]
    rot = np.append(np.c_[ct, -st], np.c_[st, ct], axis=1)
    rotv=np.einsum("...ij,...jk", rot, v.reshape((1,2,1)))
    return rotv[...,0]

def drawlocal(path, Rport, Rstarboard, Rstern, N=1):
    base=path[::N,:2]
    heading=np.c_[cos(path[::N,2]), sin(path[::N,2])]

    lport = path2local(path, Rport)
    lstarboard = path2local(path, Rstarboard)
    lstern = path2local(path, Rstern)

    fig=pylab.figure(3)
    fig.clear()
    ax=fig.add_subplot(111)
    ax.set_aspect(1)
    ax.quiver(base[:,0], base[:,1], heading[:,0], heading[:,1])

    portbase = base+rot(path[::N,2:3],Rport)
    portsteer = rot(path[::N,2:3]+lport[::N,0:1], np.r_[1.0, 0.0])
    ax.quiver(portbase[:,0], portbase[:,1], portsteer[:,0], portsteer[:,1],
            color="r")
    starboardbase = base+rot(path[::N,2:3],Rstarboard)
    starboardsteer = rot(path[::N,2:3]+lstarboard[::N,0:1], np.r_[1.0, 0.0])
    ax.quiver(starboardbase[:,0], starboardbase[:,1], starboardsteer[:,0],
            starboardsteer[:,1],
            color="g")
    sternbase = base+rot(path[::N,2:3],Rstern)
    sternsteer = rot(path[::N,2:3] + lstern[::N,0:1], np.r_[1.0, 0.0])
    ax.quiver(sternbase[:,0], sternbase[:,1], sternsteer[:,0], sternsteer[:,1],
            color="b")

    fig.canvas.draw()

