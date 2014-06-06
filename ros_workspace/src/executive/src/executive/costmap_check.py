#!/usr/bin/env python
import math
import collections
import threading
import random
import numpy as np

import rospy
import tf

import nav_msgs.msg as nav_msg
import samplereturn_msgs.msg as samplereturn_msg

import samplereturn.util as util
import samplereturn.bresenham as bresenham


class ExecutiveCostmapChecker(object):
    def __init__(self):
        
        self.node_params = util.get_node_params()
        self.tf_listener = tf.TransformListener()
        self.executive_frame = self.node_params.executive_frame

        self.strafes = rospy.get_param('strafes')

        self.costmap_listener = rospy.Subscriber('local_costmap',
                                nav_msg.OccupancyGrid,
                                self.handle_costmap)
        
        self.debug_map_pub = rospy.Publisher('/test_costmap', nav_msg.OccupancyGrid)

        self.check_publisher = rospy.Publisher('costmap_check',
                                               samplereturn_msg.CostmapCheck)
        
        rospy.spin()

    def handle_costmap(self, costmap):
        if self.debug_map_pub.get_num_connections() > 0:
            self.publish_debug = True
        else:
            self.publish_debug = False
        
        #check parameters       
        lethal_threshold = self.node_params.lethal_threshold
        check_width = self.node_params.obstacle_check_width
        check_dist = self.node_params.obstacle_check_distance
        
        #load up the costmap and its info into useful structure        
        resolution = costmap.info.resolution
        origin = (np.trunc(costmap.info.width/2),
                  np.trunc(costmap.info.height/2))
        map_np = np.array(costmap.data, dtype='i1').reshape((costmap.info.height,costmap.info.width))
        actual_yaw = util.get_current_robot_yaw(self.tf_listener)
        
        #create message lists
        msg_keys = []
        blocked_keys = []
               
        for strafe_key, strafe in self.strafes.iteritems():
            
            msg_keys.append(strafe_key)
                        
            odom_yaw = actual_yaw + strafe['angle']
            
            ll = self.check_point(origin, check_width/2, (odom_yaw - math.pi/2), resolution)
            ul = self.check_point(origin, check_width/2, (odom_yaw + math.pi/2), resolution)
            lr = self.check_point(ll[0], check_dist, odom_yaw, resolution)
            ur = self.check_point(ul[0], check_dist, odom_yaw, resolution)
            
            start_points = bresenham.points(ll, ul)
            end_points = bresenham.points(lr, ur)
            
            #check lines for lethal values
            if self.any_line_blocked(start_points, end_points, lethal_threshold, map_np):
                blocked_keys.append(True)
                rospy.logdebug("PROBE SWATHE %s BLOCKED " %(strafe_key))
            else:    
                blocked_keys.append(False)
                rospy.logdebug("PROBE SWATHE %s CLEAR " %(strafe_key))
                    
            #if anything is subscribing to the test map, publish it
            if self.publish_debug:
                map_np[start_points[:,1], start_points[:,0]] = 64
                map_np[end_points[:,1], end_points[:,0]] = 64

        if self.publish_debug:
            costmap.data = list(np.reshape(map_np, -1))
            self.debug_map_pub.publish(costmap)                        
        
        costmap_check_msg = samplereturn_msg.CostmapCheck(msg_keys, blocked_keys)                    
        self.check_publisher.publish(costmap_check_msg)
        
        return
    
    def any_line_blocked(self, start_points, end_points, lethal_threshold, map_np):
        for start, end in zip(start_points, end_points):
            line = bresenham.points(start[None,:], end[None,:])
            line_vals = map_np[line[:,1], line[:,0]]
            max_val = (np.amax(line_vals))
            if np.any(line_vals > lethal_threshold):
                #for debug, mark lethal lines
                if self.publish_debug: map_np[line[:,1], line[:,0]] = 64
                #once an obstacle is found no need to keep checking this angle
                return True
        return False
    
        #returns array of array for bresenham implementation    
    def check_point(self, start, distance, angle, res):
        x = np.trunc(start[0] + (distance * math.cos(angle))/res).astype('i2')
        y = np.trunc(start[1] + (distance * math.sin(angle))/res).astype('i2')
        return np.array([[x, y]])
    