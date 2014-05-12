from math import pow
import numpy as np
import pylab

def cubicInterpolate(x0, xDot0, x1, xDot1, alpha):
    # this comes from something we worked out on the board. it could be
    # a little easier to read..
    c = -6.0 * (x1 - 0.5 * xDot1 - x0 - 0.5 * xDot0)
    b = xDot1 - xDot0 - c

    x = x0 + xDot0 * alpha + 0.5 * b * pow(alpha, 2.0) + (1.0/3.0) * c * pow(alpha, 3.0)
    v = xDot0 + b * alpha +  c * pow(alpha, 2.0)
    a = b +  2.* c * alpha
    j = 2. * c
    return x,v,a

def segtest():
    fig=pylab.figure(1)
    fig.clear()
    ax=fig.add_subplot(211)
    x=linspace(0,1)
    y=np.array([cubicInterpolate(0, 0, 1, 1, a) for a in x])
    ax.plot( x, y[:,0] )

    ax=fig.add_subplot(212)
    xs=linspace(0.25,0.75)
    yq,vq,aq=cubicInterpolate(0,0,1,1,0.25)
    yt,vt,at=cubicInterpolate(0,0,1,1,0.75)
    dadt = 2
    ys=np.array([cubicInterpolate(yq, vq/dadt, yt, vt/dadt, (a-0.25)*dadt) for a in xs])
    plot(xs, ys[:,0])
    ax.plot( x, y[:,1] )
    ax.plot( xs, ys[:,1]*dadt )

    draw()
