import numpy as np
from scipy import ndimage
from scipy import linalg
import Image
import cv2

def extract_circle_colors( im, x, y, r):
    """ Find color mean and covariance inside a circle

    Intended to be a human-assisted function to determine
    color to find.
    im   image to use, (Ny, Nx, 3)
    x,y  circle center
    r    circle radius

    returns mean, covariance
    """
    # cut out bounding rectangle
    sim = im[y-r:y+r, x-r:x+r].copy()
    # make x, y grid over image
    X,Y = np.meshgrid(arange(2*r), arange(2*r))
    # mask point inside the circle
    circle_points = np.where((X-r)**2 + (Y-r)**2<r**2)
    colors = sim[circle_points]
    # coimpute mean, covariance
    mean_color = colors.mean(axis=0)
    cov_color = np.cov(colors.T)
    return mean_color, cov_color

def mahalanobis_distance(im, mean, cov):
    """ Mahalanobis distance for each pixel in im

    im    image to test (Ny, Nx, 3)
    mean  color mean (3,)
    cov   color covariance (3,3)
    returns squared mahalanobis distance
    """
    Sinv = np.linalg.inv(cov)
    diffv = (im-mean[None, None, :]).reshape((-1,3))
    distance_sq = (diffv.T*np.dot(Sinv, diffv.T)).sum(axis=0).reshape(im.shape[:2])
    return distance_sq

def find_colored_circles(image, color_mean, color_cov, tolerance=2.5):
    """ Find circular regions of a specified color

    image       image to test
    color_mean  from extract_circle_colors
    color_cov
    tolerance   sigma from mean to declare as same

    returns a list of [((x, y),r), ... ]
    """
    distance_sq = mahalanobis_distance(image, color_mean, color_cov)
    close_enough = (distance_sq<tolerance**2).astype('u1')
    filtered = cv2.morphologyEx(close_enough, cv2.MORPH_OPEN, np.ones((5,5)))
    contours,_ = cv2.findContours(filtered, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_KCOS)
    return [cv2.minEnclosingCircle(c) for c in contours]

def find_circle_size(x, y, r,
                     camera_intrinsic,
                     ground_plane_origin, ground_plane_normal,
                    ):
    """ Determine the physical size of a region

    Based on gound plane estimate and camera parameters.
    x, y, r  image center and pixel radius of region (find_colored_circles)
    camera_intrinsic  used for fx, fy, cx, cy
    ground_plane_origin  vector from camera center to ground plane origin
    ground_plane_normal  Vertical normal from ground plane. Assumed to be
                         normalized

    returns radius in meters.
    """
    # compute vector normal to plane which represents camera center
    G = ((np.dot( ground_plane_origin, ground_plane_origin)/
          np.dot( ground_plane_origin, ground_plane_normal)) *
         ground_plane_normal
        )
    # extract vertical camera intrinsics
    fy = camera_intrinsic[1,1]
    cy = camera_intrinsic[1,2]
    # vertical angle to object
    psi = np.arctan2(y-cy, fy)
    fx = camera_intrinsic[0,0]
    cx = camera_intrinsic[0,2]
    # horizontal angle to object
    phi = np.arctan2(x-cx, fx)
    # vertical angle from camera z-axis to G
    theta = np.arctan2(-G[2], G[0])
    # Distance from camera to ground plane in object direction
    R = np.cos(phi)*linalg.norm(G)/np.cos(psi+theta)
    return r*R/fx

# im_name = "IMG_8400.JPG"
# im = np.array(Image.open(im_name))
# im_luma = ((im*np.r_[0.2126, 0.7152, 0.0722][None, None, :]).sum(axis=2)).astype('u1')
# im_grey = (im.sum(axis=2)/3.).astype('u1')
# sim = im[2540:2700,1900:2100]
# gsim = (sim.sum(axis=2)/3.).astype('u1')

# load an image
im2 = ndimage.imread("../Photos/IMG_8409.JPG")
# Extract a color
im2_ball_center = [2600+192+54/2, 1600+113+54/2]
im2_ball_radius = 54/2
pink_mean, pink_cov = extract_circle_colors(im2, im2_ball_center[0],
    im2_ball_center[1], im2_ball_radius)
# smaller image for testing quickly
test_im2 = im2[im2_ball_center[1]-200:im2_ball_center[1]+100,im2_ball_center[0]-200:im2_ball_center[0]+100]

# camera parameters
camera_pose = np.eye(4)
camera_intrinsic = np.zeros((3,3))
# 18mm focal length and 4.3 micron pixels
f=0.018/4.3e-6
camera_intrinsic[0,0] = f
camera_intrinsic[0,2] = test_im2.shape[1]/2.
camera_intrinsic[1,1] = f
camera_intrinsic[1,2] = test_im2.shape[0]/2.
camera_intrinsic[2,2] = 1.0

# ground plane example
ground_plane_origin = np.r_[0, 0, -1.45]
tilt = 0.35
ground_plane_normal = np.r_[-sin(tilt), 0, cos(tilt)]

# find in oriinal image
circs2 = find_colored_circles(test_im2, pink_mean, pink_cov)
cs2 = [((x,y), find_circle_size(x, y, r,
                        camera_intrinsic,
                        ground_plane_origin, ground_plane_normal))
       for ((x,y),r) in circs2]

# find in another image
im3 = ndimage.imread("../Photos/IMG_8408.JPG")
camera_intrinsic[0,2] = im3.shape[1]/2.
camera_intrinsic[1,2] = im3.shape[0]/2.
circs3 = find_colored_circles(im3, pink_mean, pink_cov)
cs3 = [((x,y), find_circle_size(x, y, r,
                        camera_intrinsic,
                        ground_plane_origin, ground_plane_normal))
       for ((x,y),r) in circs3]

