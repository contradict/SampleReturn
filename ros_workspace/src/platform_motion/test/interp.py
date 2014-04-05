def hermite00(s):
    return (1.0+2.0*s)*(1.0-s)*(1.0-s)

def hermite00p(s):
    return 2.0*(1.0-s)*(1.0-s) - (1.0+2.0*s)*2*(1.0-s)

def hermite01(s):
    return s*s*(3.0-2.0*s)

def hermite01p(s):
    return 2*s*(3.0-2.0*s) - 2*s*s

def hermite10(s):
    return s*(1.0-s)*(1.0-s)

def hermite10p(s):
    return (1.0-s)*(1.0-s) - 2*s*(1.0-s)

def hermite11(s):
    return s*s*(s-1.0)

def hermite11p(s):
    return 2*s*(s-1.0) + s*s

def cubic_interp(p0, v0, p1, v1, s):
    p=p0*hermite00(s) + v0*hermite10(s) + p1*hermite01(s) + v1*hermite11(s)
    #v=p0*hermite00p(s) + v0*hermite10p(s) + p1*hermite01p(s) + v1*hermite11p(s)
    v=v0*(1.0-s) + v1*s
    return (p,v)

def velocities(ps, ts, v0):
    if len(ts)>2:
        vs = concatenate((
            [v0],
            (ps[2:]-ps[1:-1])/(ts[2:]-ts[1:-1])/2+(ps[1:-1]-ps[:-2])/(ts[1:-1]-ts[:-2])/2,
            [0]))
    elif len(ts)>1:
        vs = [v0, 0]
    else:
        raise Exception("Must have at least 2 points to find velocity")
    return vs

def interpolate(xs, ys, thetas, times, vxs, vys, omegas, t):
    if t<times[0] :
        raise Exception("Interpolation before start requested")
    if t>times[-1] :
        raise Exception("Interpolation after end requested")
    i=times.searchsorted(t)
    # obey input boundary conditions
    if i==0 :
        return (xs[0],vxs[0]), (ys[0],vys[0]), (thetas[0],omegas[0])
    if i==len(times) :
        return (xs[-1],vxs[-1]), (ys[-1],vys[-1]), (thetas[-1],omegas[-1])
    s=(t-times[i-1])/(times[i]-times[i-1])
    x, vx=cubic_interp(xs[i-1], vxs[i-1], xs[i], vxs[i], s)
    #vx/=(times[i]-times[i-1])
    y, vy=cubic_interp(ys[i-1], vys[i-1], ys[i], vys[i], s)
    #vy/=(times[i]-times[i-1])
    theta, omega=cubic_interp(thetas[i-1], omegas[i-1], thetas[i], omegas[i], s)
    #omega/=(times[i]-times[i-1])
    return (x,vx), (y,vy), (theta,omega)

t=linspace(0,10,10)
dt=t[1]-t[0]
v=1-(cos(t*2*pi/10)+1)/2
thetas=(t/10)**2
xs=cumsum(v*cos(thetas)*dt)
ys=cumsum(v*sin(thetas)*dt)
vxs=velocities(xs,t,0)
vys=velocities(ys,t,0)
omegas=velocities(thetas,t,0)
t2=linspace(0, 10, 100)
ixs=[]
ivxs=[]
iys=[]
ivys=[]
ithetas=[]
iomegas=[]
for ti in t2:
    (ix,vx),(iy,vy),(itheta,omega)= interpolate(xs, ys, thetas, t, vxs, vys,
            omegas, ti)
    ixs.append(ix)
    ivxs.append(vx)
    iys.append(iy)
    ivys.append(vy)
    ithetas.append(itheta)
    iomegas.append(omega)
figure(1)
clf()
title("position")
plot(xs,ys,label="original")
plot(ixs, iys, label="interpolated")
legend()
figure(2)
clf()
title("velocity")
plot(t, vxs, label="original")
plot(t2, ivxs, label="interpolated")
legend()
draw()

