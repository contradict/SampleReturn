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

        self.strafes = rospy.get_param('strafes')

        self.debug_map_pub = rospy.Publisher('/test_costmap', nav_msg.OccupancyGrid)

        self.costmap_listener = rospy.Subscriber('local_costmap',
                                nav_msg.OccupancyGrid,
                                self.handle_costmap)
        
        self.check_publisher = rospy.Publisher('costmap_check',
                                               samplereturn_msg.CostmapCheck)
        
        rospy.spin()

    def world2map(self, world, info):
        origin = np.r_[info.origin.position.x,
                       info.origin.position.y]
        resolution = info.resolution
        map_coords = np.trunc((world-origin)/resolution)[::-1]
        valid = (map_coords[0]>=0 and
                 map_coords[0]<info.height and
                 map_coords[1]>=0 and
                 map_coords[1]<info.width)
        return valid, map_coords

    def publish_all_blocked(self):
        msg_keys = self.strafes.keys()
        msg_values = [True for _ in msg_keys]
        costmap_check_msg = samplereturn_msg.CostmapCheck(msg_keys, msg_values)
        self.check_publisher.publish(costmap_check_msg)

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
        try:
            pos, quat = self.tf_listener.lookupTransform(costmap.header.frame_id,
                                                'base_link',
                                                rospy.Time(0))
        except tf.Exception, e:
            rospy.logerr("Unable to look up transform base_link->%s, reporting all blocked",
                    costmap.header.frame_id)
            self.publish_all_blocked()
            return
        actual_yaw = tf.transformations.euler_from_quaternion(quat)[-1]
        valid, origin = self.world2map(pos[:2], costmap.info)
        if not valid:
            rospy.logerr("Robot off costmap, cannot check. Returning all blocked.")
            rospy.logerr("Frame: %s point: %s map: %s-%s",
                    costmap.header.frame_id,
                    pos[:2],
                    (costmap.info.origin.position.x,costmap.info.origin.position.y),
                    (costmap.info.origin.position.x+costmap.info.width*costmap.info.resolution,
                     costmap.info.origin.position.y+costmap.info.height*costmap.info.resolution))
            self.publish_all_blocked()
            return

        map_np = np.array(costmap.data, dtype='i1').reshape((costmap.info.height,costmap.info.width))
        
        #create message lists
        msg_keys = []
        blocked_keys = []
               
        for strafe_key, strafe in self.strafes.iteritems():
            
            msg_keys.append(strafe_key)
                        
            odom_yaw = actual_yaw + strafe['angle']
            
            right_offset = strafe.get('right_check_offset', 0)
            left_offset = strafe.get('left_check_offset', 0)
            
            ll = self.check_point(origin,
                                  check_width/2 + right_offset,
                                  (odom_yaw - math.pi/2),
                                  costmap.info.resolution)
            ul = self.check_point(origin,
                                  check_width/2 + left_offset,
                                  (odom_yaw + math.pi/2),
                                  costmap.info.resolution)
            lr = self.check_point(ll[0], check_dist, odom_yaw, costmap.info.resolution)
            ur = self.check_point(ul[0], check_dist, odom_yaw, costmap.info.resolution)
            
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
    
