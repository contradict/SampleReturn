#!/usr/bin/env python
import roslib; roslib.load_manifest('global_path')

import sys

import rospy
from global_path.srv import *
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
import path

MAP_DIM = 600

def gridcells_to_image(gridcells): # TEMPORARY HACK! Convert path.py to use gridcells directly
    image = np.zeros((MAP_DIM, MAP_DIM))
    for cell in gridcells:
        image[cell.y, cell.x] = 1
    return image

def generate_path():
    rospy.wait_for_service('mock_fence')
    try:
        mock_fence = rospy.ServiceProxy('mock_fence', MockFence)

        robot_x = 148
        robot_y = 400
        
        plt.xlim(0,MAP_DIM)
        plt.ylim(0,MAP_DIM)
        plt.scatter(robot_x, robot_y)
        path_x = []
        path_y = []
        for i in range(100):
            resp1 = mock_fence(robot_x, robot_y)
            image = gridcells_to_image(resp1.map.cells)
            wp = path.get_next_waypoint(path.Point(robot_x, robot_y), image)
            print wp.x, wp.y
            path_x.append(wp.x)
            path_y.append(wp.y)
            robot_x = wp.x
            robot_y = wp.y
        plt.plot(path_x, path_y, c= 'r')
        plt.imshow(image, origin='lower', cmap = cm.Greys_r)
        plt.show()
        return None
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
if __name__ == "__main__":
    generate_path()