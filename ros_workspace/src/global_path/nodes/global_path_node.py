#!/usr/bin/env python
import roslib; roslib.load_manifest('global_path')

import sys

import rospy
from global_path.srv import *
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
import path as pathlib

MAP_DIM = 600

def gridcells_to_image(gridcells): # TEMPORARY HACK! Convert path.py to use gridcells directly
    image = np.zeros((MAP_DIM, MAP_DIM))
    for cell in gridcells:
        image[cell.y, cell.x] = 1
    return image

def path_loop():
    # TODO - get robot position from localization
    start = pathlib.Point(148, 400)
    loopstart_index = 0
    robot = start
    path = [robot]
    
    plt.xlim(0,MAP_DIM)
    plt.ylim(0,MAP_DIM)
    plt.scatter(robot.x, robot.y)
        
    rospy.wait_for_service('mock_fence')
    try:
        mock_fence = rospy.ServiceProxy('mock_fence', MockFence)
        
        LOOP_END_DIST = 10
        # first exploration loop
        while True:
            resp1 = mock_fence(robot.x, robot.y)
            image = gridcells_to_image(resp1.map.cells)
            waypoint, alongFence = pathlib.get_next_waypoint(robot, image)
            if not alongFence:
                loopstart_index += 1
            print waypoint.x, waypoint.y
            path.append(waypoint)
            robot = waypoint
            if len(path)-loopstart_index > 10 and path[loopstart_index].dist(robot) < LOOP_END_DIST:
                path.append(path[loopstart_index])
                break
            elif len(path) > 300: # TODO DEBUG ONLY REMOVE
                print "I think I'm infinite looping aaaaaagh break"
                break
            
        plt.plot([p.x for p in path], [p.y for p in path], ':')
        plt.imshow(image, origin='lower', cmap = cm.Greys_r)
        plt.show()
    
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
if __name__ == "__main__":
    path_loop()