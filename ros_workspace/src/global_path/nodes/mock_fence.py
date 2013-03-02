#!/usr/bin/env python
import roslib; roslib.load_manifest('global_path')

from global_path.srv import *
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point
import rospy

import Image
import numpy as np
from collections import namedtuple

class Range(object):
    def __init__(self, min, max):
        self.min = min
        self.max = max
        
view_x = Range(min=600, max=0)
view_y = Range(min=600, max=0)

def load_image(filename):
    image = Image.open(filename).convert("L")
    image = np.asarray(image)
    return image

def adjust_view(x, y):
    global view_x, view_y
    SIGHT = 20
    if x - SIGHT < view_x.min:
        view_x.min = max(0, x-SIGHT)
    if x + SIGHT > view_x.max:
        view_x.max = min(600, x+SIGHT)
    if y - SIGHT < view_y.min:
        view_y.min = max(0, y-SIGHT)
    if y + SIGHT > view_y.max:
        view_y.max = min(600, y+SIGHT)
    print "My view range is %d->%d x and %d->%d y" % (view_x.min,
                                                      view_x.max,
                                                      view_y.min,
                                                      view_y.max)
        
def image_to_gridcells(image):
    cells = []
    subset = image[view_y.min:view_y.max, view_x.min:view_x.max]
    for y, row in enumerate(subset):
        for x, column in enumerate(row):
            true_x = x + view_x.min
            true_y = y + view_y.min
            if subset[y][x]:
                cells.append(Point(x=true_x, y=true_y))
    return GridCells(cells=cells)

def generate_gridcells(req):
    print "I see the robot is at %d,%d" % (req.robot_x, req.robot_y)
    adjust_view(req.robot_x, req.robot_y)
    print "Loading image"
    image = load_image('corridor.png')
    print "Converting image to gridcells"
    gridcells = image_to_gridcells(image)
    return gridcells

def mock_fence_server():
    rospy.init_node('mock_fence')
    s = rospy.Service('mock_fence', MockFence, generate_gridcells)
    
    print "Ready to generate gridcells."
    rospy.spin()

if __name__ == "__main__":
    mock_fence_server()
