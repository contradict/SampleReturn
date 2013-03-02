import numpy as np
import math

def norm(vector):
    """ Returns the norm (length) of the vector."""
    # note: this is a very hot function, hence the odd optimization
    # Unoptimized it is: return np.sqrt(np.sum(np.square(vector)))
    return np.sqrt(np.dot(vector, vector))

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    angle = np.arccos(np.dot(v1_u, v2_u))
    if math.isnan(angle):
        if (v1_u == v2_u).all():
            return 0.0
        else:
            return np.pi
    return angle

#http://www.bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
def ccw(A,B,C):
    return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x)

# Return true if line segments AB and CD intersect
def intersect(A,B,C,D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

# http://people.virginia.edu/~ll2bf/docs/various/polyarea.html
def simpoly(x,y):
  """
  A function that calculates the area of a 2-D simple polygon (no matter concave or convex)
  Must name the vertices in sequence (i.e., clockwise or counterclockwise)
  Square root input arguments are not supported
  Formula used: http://en.wikipedia.org/wiki/Polygon#Area_and_centroid
  Definition of "simply polygon": http://en.wikipedia.org/wiki/Simple_polygon

  Input x: x-axis coordinates of vertex array
        y: y-axis coordinates of vertex array
  Output: polygon area
 """

  ind_arr = np.arange(len(x))-1  # for indexing convenience
  s = 0
  for ii in ind_arr:
    s = s + (x[ii]*y[ii+1] - x[ii+1]*y[ii])

  return abs(s)*0.5