""" VFH Move Server
Use vfh to drive to a position and yaw
"""
#!/usr/bin/env python
import numpy as np
from copy import deepcopy
import rospy
import actionlib
from tf.transformations import euler_from_quaternion
import tf

import std_msgs.msg as std_msg
import nav_msgs.msg as nav_msg
import visualization_msgs.msg as vis_msg
import geometry_msgs.msg as geometry_msg


from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, PoseStamped, Point, PointStamped, Quaternion
from samplereturn_msgs.msg import (VFHMoveAction,
                                   VFHMoveResult,
                                   VFHMoveFeedback)
from samplereturn_msgs.srv import ObstacleCheck, ObstacleCheckRequest
from motion_planning.simple_motion import SimpleMover

import samplereturn.util as util
import samplereturn.bresenham as bresenham

def euler_from_orientation(orientation):
    """
    euler angles from geometry_msgs/Quaterion
    """
    return euler_from_quaternion((
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
        ))

class VFHMoveServer( object ):
    """
    VFHMoveServer node
    """
    def __init__(self):
        
        self.publish_debug=True
        
        self._goal_orientation_tolerance = \
                rospy.get_param("~goal_orientation_tolerance", 0.05)
        self._lethal_threshold = rospy.get_param("~lethal_threshold", 100)
        self.odometry_frame = rospy.get_param("~odometry_frame")
        self._mover = SimpleMover("~simple_motion_params/")
        
        #calculate the stop_distance for stopping once near target pose
        accel = rospy.get_param("~simple_motion_params/acceleration")
        max_vel = rospy.get_param("~simple_motion_params/max_velocity")
        self.stop_distance = max_vel**2/2./accel

        self._as = actionlib.SimpleActionServer("vfh_move", VFHMoveAction,
                execute_cb = self.execute_cb, auto_start=False)

        self._tf = tf.TransformListener()

        self._goal_odom = None

        #costmap crap
        self.costmap = None
        
        self.debug_map_pub = rospy.Publisher('/test_costmap',
                                             nav_msg.OccupancyGrid,
                                             queue_size=1)

        self.costmap_listener = rospy.Subscriber('local_costmap',
                                nav_msg.OccupancyGrid,
                                self.handle_costmap)
        
        self.costmap_info = None
        self.costmap_header = None
 
        #vfh params
        self.vfh_running = False

        self.sector_angle = np.radians(5.0)
        self.active_window = np.radians(120)
        #sector count, 1 more than window/angle
        self.sector_count = int(self.active_window/self.sector_angle) + 1
        #offset from first sector in array to the robot zero sector
        self.zero_offset = int(self.active_window/(self.sector_angle * 2))
        #sectors are free if 0, or blocked if 1, start with all free!
        self.sectors = np.zeros(self.sector_count, dtype=np.bool)
        
        self.min_obstacle_distance = 1.5 #minimum distance at which to check for obstacles
        
        #constants a and b:  vfh guy say they should be set such that a-b*dmax = 0
        #so that obstacles produce zero cost at a range of dmax
        self.max_obstacle_distance = 6.0  #this is dmax
        self.const_b = 1 #uuuuh, if this is 1 then a = dmax... is that cool?
        self.const_a = self.const_b*self.max_obstacle_distance
        
        #cost coefficients
        self.u_goal = 5 #cost multiplier for angular difference to target_angle
        self.u_current = 3 #cost multiplier for angular difference from current angle
        
        #thresholds at which to set clear, or blocked, a gap for hysteresis
        #I think lethal cells are set to 100 in costmap, so these thresholds will be kinda high
        self.threshold_high = 4000
        self.threshold_low = 2000
        
        #target point and current_yaw
        self.target_point = None
        self.current_sector_index = 0 #start moving straight along the robot's yaw
        
        #debug marker stuff
        self.debug_marker_pub = rospy.Publisher('vfh_markers',
                                                vis_msg.MarkerArray,
                                                queue_size=2)
        self.marker_id = 0

        #this loop controls the strafe angle using vfh sauce
        rospy.Timer(rospy.Duration(0.5), self.vfh_planner)
        
        self.obstacle_check_service = rospy.Service('obstacle_check',
                                                    ObstacleCheck,
                                                    self.obstacle_check)        
        
        self._as.start()

    def execute_cb(self, goal):
        """
        Called by SimpleActionServer when a new goal is ready
        """
        try:
            rospy.loginfo("VFH_mover target_goal: %s" % goal.target_pose)
            goal_local = self._tf.transformPose('base_link', goal.target_pose)
            goal_odom = self._tf.transformPose(self.odometry_frame, goal.target_pose)
        except tf.Exception, exc:
            rospy.logwarn("VFH MOVE SERVER:could not transform goal: %s"%exc)
            self._as.set_aborted(text = "Could not transform goal: %s"%exc)
            return
        self._goal_odom = goal_odom
        rospy.logdebug("Received goal, transformed to %s", goal_odom)

        # turn to face goal
        dyaw = np.arctan2( goal_local.pose.position.y,
                           goal_local.pose.position.x)
        if np.abs(dyaw) > self._goal_orientation_tolerance:
            rospy.logdebug("Rotating by %f.", dyaw)
            self._mover.execute_spin(dyaw,
                    stop_function=self._as.is_preempt_requested)
            if self._as.is_preempt_requested():
                rospy.logdebug("Preempted during initial rotation.")
                self._as.set_preempted()
                return

        # drive to goal using vfh
        distance = np.hypot(goal_local.pose.position.x,
                goal_local.pose.position.y)
        #use param velocity unless a velocity was included in goal
        velocity = goal.velocity if (goal.velocity != 0) else None
        rospy.logdebug("Moving %f meters ahead.", distance)
        self.vfh_running = True
        self._mover.execute_continuous_strafe(0.0,
                max_velocity = velocity,
                stop_function=self._as.is_preempt_requested)
        self.vfh_running = False
        if self._as.is_preempt_requested():
            rospy.logdebug("Preempted during drive to goal.")
            self._as.set_preempted()
            return

        # turn to requested orientation
        dyaw = self.yaw_error()
        if np.abs(dyaw) > self._goal_orientation_tolerance:
            rospy.logdebug("Rotating to goal yaw %f.", dyaw)
            self._mover.execute_spin(dyaw,
                    stop_function=self._as.is_preempt_requested)
            if self._as.is_preempt_requested():
                rospy.logdebug("Preempted during final rotation.")
                self._as.set_preempted()
                return
        else:
            rospy.logdebug("Within tolerance of goal yaw, not rotating.")

        rospy.logdebug("Successfully completed goal.")
        self._as.set_succeeded(
                VFHMoveResult(True,
                self.position_error(),
                Float64(self.yaw_error())))
        
    def vfh_planner(self, event):
        if not self.vfh_running:
            return
        
        start_time = rospy.get_time()

        #get current position from mover object
        robot_position = self._mover.current_position
        robot_yaw = self._mover.current_yaw

        valid, robot_cmap_coords = self.world2map(robot_position[:2], self.costmap_info)

        target_in_odom = PointStamped(self._goal_odom.header,
                                      self._goal_odom.pose.position)
        yaw_to_target, distance_to_target = util.get_robot_strafe(self._tf, target_in_odom)
        target_index = round(yaw_to_target/self.sector_angle) + self.zero_offset
        
        #if we are within stop_distance initiate stop
        if distance_to_target < self.stop_distance:
            self.mover.stop()
                    
        #if the target is not in the active window, we are probably way off course,
        #or possibly trying to drive too far around an obstacle near the target: stop!
        if not 0 <= target_index < len(self.sectors):
            self._mover.stop()
            if target_index < 0: target_index = 0
            if target_index >= len(self.sectors): target_index = len(self.sectors)
        
        obstacle_density = np.zeros(self.sector_count)
        inverse_cost = np.zeros(self.sector_count)
        #nonzero_coords = np.transpose(np.nonzero(self.costmap))        

        for index in range(len(self.sectors)):
            #yaw in odom            
            sector_yaw = robot_yaw + (index - self.zero_offset) * self.sector_angle
            #get start and end points for the bresenham line
            start = self.bresenham_point(robot_cmap_coords,
                                        self.min_obstacle_distance,
                                        sector_yaw,
                                        self.costmap_info.resolution)
            end = self.bresenham_point(robot_cmap_coords,
                                        min(self.max_obstacle_distance, distance_to_target),                                   
                                        sector_yaw,
                                        self.costmap_info.resolution)            
            sector_line = bresenham.points(start, end)

            for coords in sector_line:
                #coords = coords[::-1]
                cell_value = self.costmap[tuple(coords)]

                position = self.costmap_info.resolution * (coords - robot_cmap_coords)
                distance = np.linalg.norm(position)
            
                #m is the obstacle vector magnitude
                m = cell_value * (self.const_a - self.const_b * distance)
                
                obstacle_density[index] += m
            
            #set sectors above threshold density as blocked, those below as clear, and do
            #not change the ones in between
            if obstacle_density[index] >= self.threshold_high:
                self.sectors[index] = True
                inverse_cost[index] = 0 #inversely infinitely expensive
            if obstacle_density[index] <= self.threshold_low:
                self.sectors[index] = False
            #if sector is clear (it may not have been changed this time!) set its cost, making it a candidate sector
            if not self.sectors[index]:
                inverse_cost[index] = 1.0 / ( self.u_goal*np.abs(yaw_to_target - (index - self.zero_offset)*self.sector_angle)
                                            + self.u_current*np.abs(self.current_sector_index - index)*self.sector_angle) 

        #stop the mover if the path is blocked.  This is an estop situation!
        if np.all(self.sectors):
            self._mover.estop()

        #find the sector index with the lowest cost index (inverse cost!)
        min_cost_index = np.argmax(inverse_cost)
        if self.current_sector_index != min_cost_index:
            self._mover.set_strafe_angle( (min_cost_index - self.zero_offset) * self.sector_angle)
            self.current_sector_index = min_cost_index

        #START DEBUG CRAP
        if self.publish_debug:
    
            rospy.logdebug("TARGET SECTOR index: %d" % (target_index))
            rospy.logdebug("SELECTED SECTOR index: %d" %(self.current_sector_index))
            rospy.logdebug("SECTORS: %s" % (self.sectors))
            rospy.logdebug("OBSTACLES: %s" % (obstacle_density))
            rospy.logdebug("INVERSE_COSTS: %s" % (inverse_cost))
            
            vfh_debug_array = []
            vfh_debug_marker = vis_msg.Marker()
            vfh_debug_marker.pose = Pose()
            vfh_debug_marker.header = std_msg.Header(0, rospy.Time(0), self.odometry_frame)
            vfh_debug_marker.type = vis_msg.Marker.ARROW
            vfh_debug_marker.color = std_msg.ColorRGBA(0, 0, 0, 1)
            vfh_debug_marker.scale = geometry_msg.Vector3(min(self.max_obstacle_distance, distance_to_target), .02, .02)
            vfh_debug_marker.lifetime = rospy.Duration(0.5)
            
            #rospy.loginfo("SECTORS: %s" % (self.sectors))
            for index in range(len(self.sectors)):
                debug_marker = deepcopy(vfh_debug_marker)
                sector_yaw = robot_yaw + (index - self.zero_offset) * self.sector_angle
                debug_marker.pose.orientation = geometry_msg.Quaternion(*tf.transformations.quaternion_from_euler(0, 0, sector_yaw))
                debug_marker.pose.position = geometry_msg.Point(robot_position[0], robot_position[1], 0)
                if self.current_sector_index == index:
                    debug_marker.color = std_msg.ColorRGBA(0.1, 1.0, 0.1, 1)                
                elif not self.sectors[index]:
                    debug_marker.color = std_msg.ColorRGBA(0.1, 0.1, 1.0, 1)                
                else:
                    debug_marker.color = std_msg.ColorRGBA(1.0, 0.1, 0.1, 1)                
                debug_marker.id = self.marker_id
                self.marker_id += 1
                vfh_debug_array.append(debug_marker)
                
            debug_marker = deepcopy(vfh_debug_marker)
            target_yaw_odom = robot_yaw + yaw_to_target
            debug_marker.pose.orientation = geometry_msg.Quaternion(*tf.transformations.quaternion_from_euler(0, 0, target_yaw_odom))
            debug_marker.pose.position = geometry_msg.Point(robot_position[0], robot_position[1], 0)
            debug_marker.color = std_msg.ColorRGBA(1.0, 0.1, 1.0, 1)                
            debug_marker.id = self.marker_id
            self.marker_id += 1
            vfh_debug_array.append(debug_marker)    
    
            vfh_debug_msg = vis_msg.MarkerArray(vfh_debug_array)
            self.debug_marker_pub.publish(vfh_debug_msg)
        
            #rospy.loginfo("VFH LOOP took: %.4f seconds to complete" % (rospy.get_time() - start_time))        

    def handle_costmap(self, costmap):
               
        #load up the costmap and its info into useful structure        
        
        self.costmap_info = costmap.info
        self.costmap_header = costmap.header
        
        self.costmap = np.array(costmap.data,
                                dtype='i1').reshape((costmap.info.height,
                                                     costmap.info.width))
        
    def obstacle_check(self, request ):
        """
        Check a rectangular area in the costmap and return true if any lethal
        cells are inside that area.
        """
        
        #get current position from mover object
        robot_position = self._mover.current_position
        robot_yaw = self._mover.current_yaw
        valid, robot_cmap_coords = self.world2map(robot_position[:2], self.costmap_info)
        
        ll = self.bresenham_point(robot_cmap_coords,
                                  request.width/2,
                                  (robot_yaw - np.pi/2),
                                  self.costmap_info.resolution)
        ul = self.bresenham_point(robot_cmap_coords,   
                                  request.width/2,
                                  (robot_yaw + np.pi/2),
                                  self.costmap_info.resolution)
        lr = self.bresenham_point(ll[0], request.distance, robot_yaw, self.costmap_info.resolution)
        ur = self.bresenham_point(ul[0], request.distance, robot_yaw, self.costmap_info.resolution)

        start_points = bresenham.points(ll, ul)
        end_points = bresenham.points(lr, ur)
        
        #check lines for lethal values
        if self.any_line_blocked(start_points, end_points):
            rospy.logdebug("PROBE SWATHE %f X %f BLOCKED " %(request.width, request.distance))
            return True
        else:    
            rospy.logdebug("PROBE SWATHE %f X %f CLEAR " %(request.width, request.distance))
            return False
        
    def any_line_blocked(self, start_points, end_points):
        for start, end in zip(start_points, end_points):
            line = bresenham.points(start[None,:], end[None,:])
            line_vals = self.costmap[line[:,1], line[:,0]]
            max_val = (np.amax(line_vals))
            if np.any(line_vals > self._lethal_threshold):
                #for debug, mark lethal lines
                if self.publish_debug: self.costmap[line[:,1], line[:,0]] = 64
                #once an obstacle is found no need to keep checking this angle
                return True
        return False
        
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

    #returns a 1d array containing the coordinates array for our bresenham implementation    
    def bresenham_point(self, start, distance, angle, res):
        x = np.trunc(start[1] + (distance * np.cos(angle))/res).astype('i2')
        y = np.trunc(start[0] + (distance * np.sin(angle))/res).astype('i2')
        return np.array([[y, x]])

    def yaw_error(self):
        """
        Compute difference between present orientation and
        goal orientation.
        """
        current_pose = util.get_current_robot_pose(self._tf,
                                                   self.odometry_frame)             
        
        return (euler_from_orientation(self._goal_odom.pose.orientation)[-1] -
            euler_from_orientation(current_pose.pose.orientation)[-1])

    def position_error(self):
        """
        Compute difference between present position and
        goal position.
        """
        
        current_pose = util.get_current_robot_pose(self._tf,
                                                   self.odometry_frame)          
        return Point(
                self._goal_odom.pose.position.x -
                current_pose.pose.position.x,
                self._goal_odom.pose.position.y -
                current_pose.pose.position.y,
                0)

    def publish_feedback(self):
        """
        Send actionserver feedback.
        """
        self._as.publish_feedback(
                VFHMoveFeedback( self.position_error(),
                    Float64(self.yaw_error())
                    )
                )

