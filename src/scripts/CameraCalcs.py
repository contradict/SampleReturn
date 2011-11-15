
def Dsquare(yi, h, f, psi):
    A = (1 + (yi/f)**2)
    B = (yi/f)*cos(psi) + sin(psi)
    return h**2*(A/B**2 - 1)

def dDdyi(yi, h, f, psi):
    A = (1 + (yi/f)**2)
    B = (yi/f)*cos(psi) + sin(psi)
    dAdyi = 2*yi/f**2
    dBdyi = cos(psi)/f
    D = sqrt(Dsquare(yi, h, f, psi))
    return (h**2/2/D)*(dAdyi/B**2 - 2*A/B**3*dBdyi)

# def dDdyi(yi, h, f, psi, D):
#     A = 2*(h/f)**2*yi
#     B = (yi/f)*cos(psi) + sin(psi)
#     C = 2*h**2*(1+(yi/f)**2)*cos(psi)/f
#     return (A/B**2 - C/B**3)/D/2.0

def print_details(
                  f=0.004,
                  h=1.0,
                  Nx=752, Ny=480,
                  Sw=0.00451, Sh=0.00288,
                  pw=6e-6,
                  D=20.0,
                  N=2.0,
                  ):
    
    """ Print calcualtions for camera position

    f       Camera focal length
    h       Camara height above ground
    Nx, Ny  Sensor Pixels horizontal, vertical
    Sw, Sh  Sensor dimensions horizontal, vertical
    pw      Pixel size in meters. not checked against above
    D       Longest interesting diatance
    N       distance to near edge
    """

    print "Lens focal length %4.1fmm"%(f*1000.)
    Hfov = 2*arctan(Sw/f/2)
    Vfov = 2*arctan(Sh/f/2)
    print "Hfov = %5.1f degrees"%degrees(Hfov)
    print "Vfov = %5.1f degrees"%degrees(Vfov)

    # top of view frustum horizontal
    # psi = Vfov/2
    # near edge at N
    psi = arctan(h/N) - Vfov/2
    print "%3.2fm camera elevation with %5.2f degree down-pitch"%(h, degrees(psi))
    print "Near edge at %3.2fm"%(h/tan(psi+Vfov/2))
    top = (psi-Vfov/2)
    if top<=0:
        print "Top edge is horizontal or higher: %4.1f degrees"%degrees(top)
    else:
        print "Top edge at %3.2fm"%(h/tan(psi-Vfov/2))
    Y_D = round(scipy.optimize.fsolve(lambda x:Dsquare(x*pw, h, f, psi) - D**2,
                                      0, # initial guess
                                      fprime=lambda x:dDdyi(x*pw, h, f, psi))[0])
    if Y_D > 0:
        dir = 'below'
    else:
        dir = 'above'
    if abs(Y_D)>Ny/2:
        print "ERROR: %3.1fm is beyond top edge of sensor (Y_D=%d)"%(D, Y_D)
    else:
        print "%3.1fm at %d pixels %s center"%(D, abs(Y_D), dir)
        print "At %3.1fm, 1 vertical pixel is %7.4fm on the ground"%(D,
            (-1.0*pw)*dDdyi(Y_D*pw, h, f, psi))
    top_row = max(Y_D, -Ny/2)
    yi_r = linspace(top_row, Ny/2, Ny/2-top_row + 1)
    fig = pylab.figure(1)
    fig.clf()
    ax = fig.add_subplot(211)
    ax.plot(yi_r, -dDdyi(yi_r*pw, h, f, psi)*pw)
    ax.set_ylabel('distance per pixel\non ground')
    ax = fig.add_subplot(212, sharex=ax)
    ax.plot(yi_r, sqrt(Dsquare(yi_r*pw, h, f, psi)))
    ax.set_xlabel('Vertical pixels (0 at center)')
    ax.set_ylabel('Ground Distance\nfrom Camera')

    print "At %3.1fm, 1 horizontal pixel is %7.4fm on the ground"%(D, (1.0*pw)*D/f)
 
