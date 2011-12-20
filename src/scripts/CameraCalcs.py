import scipy
import scipy.optimize

def Dsquare(yi, h, f, psi):
    A = (1 + (yi/f)**2)
    B = (yi/f)*cos(psi) + sin(psi)
    return h**2*(A/B**2 - 1)

def dDsqdyi(yi, h, f, psi):
    A = (1 + (yi/f)**2)
    B = (yi/f)*cos(psi) + sin(psi)
    dAdyi = 2*yi/f**2
    dBdyi = cos(psi)/f
    return h**2*(dAdyi/B**2 - 2*A/B**3*dBdyi)

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
                  rotate=False
                  ):
    
    """ Print calcualtions for camera position

    f       Camera focal length
    h       Camara height above ground
    Nx, Ny  Sensor Pixels horizontal, vertical
    Sw, Sh  Sensor dimensions horizontal, vertical
    pw      Pixel size in meters. not checked against above
    D       Longest interesting diatance
    N       distance to near edge
    rotate  If true, camera is rotated to have wide axis vertical
    """

    if rotate:
        Nx, Ny = Ny, Nx
        Sw, Sh = Sh, Sw

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
    far_edge = (h/tan(psi-Vfov/2))
    if top<=0:
        print "Top edge is horizontal or higher: %4.1f degrees"%degrees(top)
    else:
        print "Top edge at %3.2fm"%far_edge
        if far_edge<D:
            D = far_edge
    Y_D = round(scipy.optimize.fsolve(lambda x:Dsquare(x*pw, h, f, psi) - D**2,
                                      -10, # initial guess
                                     fprime=lambda x:dDsqdyi(x*pw, h, f, psi)*pw)[0])
    if Y_D > 0:
        dir = 'below'
    else:
        dir = 'above'
    print "%3.1fm at %d pixels %s center"%(D, abs(Y_D), dir)
    print "At %3.1fm, 1 vertical pixel is %7.4fm on the ground"%(D,
        (-1.0*pw)*dDdyi(Y_D*pw, h, f, psi))
    top_row = max(Y_D, -Ny/2)
    yi_r = linspace(top_row, Ny/2, Ny/2-top_row + 1)
    fig = pylab.figure(1)
    fig.clf()
    ax1 = fig.add_subplot(311)
    ax1.plot(yi_r, -dDdyi(yi_r*pw, h, f, psi)*pw)
    ax1.set_ylabel('distance per pixel\non ground')
    ax1.set_ylim((0,1.0))
    ax2 = fig.add_subplot(312, sharex=ax1)
    ax2.plot(yi_r, sqrt(Dsquare(yi_r*pw, h, f, psi)))
    ax2.set_ylabel('Ground Distance\nfrom Camera')
    ax2.set_ylim((0,40.0))
    ax3 = fig.add_subplot(313, sharex=ax1)
    ax3.plot(yi_r, sqrt(Dsquare(yi_r*pw, h, f, psi) + h**2)*tan(Hfov/2)*2)
    ax3.set_ylabel('Ground Width')
    ax3.set_ylim((0,10.0))
    ax3.set_xlabel('Vertical pixels (0 at center)')
    fig.canvas.draw()

    print "At %3.1fm, 1 horizontal pixel is %7.4fm on the ground"%(D, (1.0*pw)*D/f)

# Current best guess
print_details(f=0.006, h=.145, rotate=True)
