""" VFH Move Server
Use vfh to drive to a position and yaw
"""
#!/usr/bin/env python
import numpy as np
import threading
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
        
        rospy.on_shutdown(self.shutdown_cb)
        self.shutdown=False
        
        node_params = util.get_node_params()
        self.publish_debug = node_params.publish_debug

        self.odometry_frame = node_params.odometry_frame
        self._mover = SimpleMover("~vfh_motion_params/")
        self._tf = tf.TransformListener()
 
        #action server stuff
        self._as = actionlib.SimpleActionServer("vfh_move",
                                                VFHMoveAction,
                                                auto_start=False)
        
        self._as.register_goal_callback(self.goal_cb)
        self._as.register_preempt_callback(self.preempt_cb)
        self._action_outcome = None

        #goal flags and params
        self._goal_orientation_tolerance = np.radians(node_params.goal_orientation_tolerance)
        self._goal_obstacle_radius = node_params.goal_obstacle_radius
        self._clear_distance = node_params.clear_distance
        self._goal_odom = None
        self._goal_local = None
        self._target_point_odom = None
        self._start_point = None
        self._orient_at_target = False
        self._rotate_to_clear = False
 
        #costmap crap
        self.costmap = None
        self.costmap_listener = rospy.Subscriber('local_costmap',
                                nav_msg.OccupancyGrid,
                                self.handle_costmap)
        self.costmap_info = None
        self.costmap_header = None
 
        #vfh flags
        self.vfh_running = False
        self.stop_requested = False
        self.last_vfh_update = None

        #VFH algorithm params
        self._lethal_threshold = node_params.lethal_threshold
        self.sector_angle = np.radians(node_params.sector_angle)
        self.active_window = np.radians(node_params.active_window)
        #cost coefficients
        self.u_goal = node_params.u_goal #cost multiplier for angular difference to target_angle
        self.u_current = node_params.u_current #cost multiplier for angular difference from current angle
        #constants a and b:  vfh guy say they should be set such that a-b*dmax = 0
        #so that obstacles produce zero cost at a range of dmax.  Seems like b=1 is fine...
        #so I am replacing this with a=dmax
        self.max_obstacle_distance = node_params.max_obstacle_distance #this is dmax
        self.min_obstacle_distance = node_params.min_obstacle_distance #min. dist. to consider for obstacles                
        
        #setup sectors
        self.sector_count = int(self.active_window/self.sector_angle) + 1
        #offset from first sector in array to the robot zero sector
        self.zero_offset = int(self.active_window/(self.sector_angle * 2))
        #sectors are free if 0, or blocked if 1, start with all free!
        self.sectors = np.zeros(self.sector_count, dtype=np.bool)
        self.current_sector_index = 0 #start moving straight along the robot's yaw
        self.target_point = None

        #thresholds at which to set clear, or blocked, a gap for hysteresis
        #I think lethal cells are set to 100 in costmap, so these thresholds will be kinda high
        self.threshold_high = node_params.threshold_high
        self.threshold_low = node_params.threshold_high
        
        #debug marker stuff
        self.debug_marker_pub = rospy.Publisher('vfh_markers',
                                                vis_msg.MarkerArray,
                                                queue_size=2)
        self.marker_id = 0

        #this loop controls the strafe angle using vfh sauce
        rospy.Timer(rospy.Duration(0.2), self.vfh_planner)
        
        #service to allow other nodes to look at spots in the costmap for obstacles
        self.obstacle_check_service = rospy.Service('obstacle_check',
                                                    ObstacleCheck,
                                                    self.obstacle_check)        
        
        self._as.start()

    def goal_cb(self):
        """
        Called by SimpleActionServer when a new goal is ready
        """
        #was the server active when goal was received?
        active = self._as.is_active()
        
        #this makes it active if it wasn't
        goal = self._as.accept_new_goal()
        
        try:
            goal_local = self._tf.transformPose('base_link', goal.target_pose)
            goal_odom = self._tf.transformPose(self.odometry_frame, goal.target_pose)
        except tf.Exception, exc:
            rospy.logwarn("VFH MOVE SERVER:could not transform goal: %s"%exc)
            self._as.set_aborted(result = VFHMoveResult(outcome = VFHMoveResult.ERROR))
            return
        self._goal_local = goal_local
        self._goal_odom = goal_odom
        self._orient_at_target = bool(goal.orient_at_target)
        self._rotate_to_clear = bool(goal.rotate_to_clear)
        header = std_msg.Header(0, rospy.Time(0), self.odometry_frame)
        self._target_point_odom =  PointStamped(header, self._goal_odom.pose.position)
      
        #publish the target point
        if self.publish_debug:
            self.publish_target_marker(self._target_point_odom.point)
        
        #calculate the stop_distance for stopping once near target pose
        spin_velocity = goal.spin_velocity if (goal.spin_velocity != 0) else self._mover.max_velocity
        move_velocity = goal.move_velocity if (goal.move_velocity != 0) else self._mover.max_velocity
        self.stop_distance = move_velocity**2/2./self._mover.acceleration

        #if the action server is inactive, fire up a movement thread
        if not active:
            rospy.logdebug("VFH Server not active, goal received, starting.")
            self.stop_requested = False
            self.execute_thread = threading.Thread(target=self.execute_movement,
                                                   args=(move_velocity,
                                                         spin_velocity));
            self.execute_thread.start();
        else:
            rospy.logdebug("VFH Server active, self._goal_odom updated.")
        
    #called by the AS when new goal received, and also when preempted
    def preempt_cb(self):
        #if we receive a preempt without a new goal available, this means it
        #was called by an actual preempt request
        if not self._as.is_new_goal_available():
            self.stop_requested = True
        
    def is_stop_requested(self):
        return self.stop_requested

    def execute_movement(self, move_velocity, spin_velocity):
        #store the starting point in odom
        current_pose = util.get_current_robot_pose(self._tf, self.odometry_frame)
        self._start_point = current_pose.pose.position
        #if nothing modifies this... that would be an error!
        self._action_outcome = VFHMoveResult.ERROR

        #if this flag is set, rotate until the forward sector is clear, then move
        #clear_distance straight ahead, the try to move to the target pose
        if self._rotate_to_clear:
            start_dyaw, d = util.get_robot_strafe(self._tf, self._target_point_odom)
            rot_sign = np.sign(start_dyaw)
            self._mover.execute_spin(rot_sign*2*np.pi,
                                     max_velocity = 0.1,
                                     stop_function=self.clear_check())
            if self.exit_check(): return
            dyaw, d = util.get_robot_strafe(self._tf, self._target_point_odom)
            if (start_dyaw - dyaw) < self._goal_orientation_tolerance:
                #we spun all the way around +/- a little, so we are trapped
                self._action_outcome == VFHMoveResult.BLOCKED
            else:
                #we found a clear spot, drive clear_distance meters ahead
                #save the final goal for later
                saved_odom_point = deepcopy(self._target_point_odom)
                header = std_msg.Header(0, rospy.Time(0), 'base_link')
                point = geometry_msg.PointStamped(self._clear_distance, 0, 0)
                target_point_base = geometry_msg.PointStamped(header, point)
                try:
                    self._target_point_odom = transformPoint(self.odometry_frame,
                                                             target_base_point)
                except tf.Exception, exc:
                    rospy.logwarn("VFH MOVE SERVER:could not transform point: %s"%exc)
                    self._as.set_aborted(result = VFHMoveResult(outcome = VFHMoveResult.ERROR))
                    return                
                #make the move
                self.vfh_running = True
                self._mover.execute_continuous_strafe(self.clear_distance,
                                                      max_velocity = move_velocity,
                                                      stop_function=self.is_stop_requested)
                self.vfh_running = False
                if self.exit_check(): return
                #now try to get to the target goal
                self._target_point_odom = saved_odom_point

        dyaw, d = util.get_robot_strafe(self._tf, self._target_point_odom)
        # turn to face goal
        if np.abs(dyaw) > self._goal_orientation_tolerance:
            rospy.logdebug("VFH Rotating to heading by %f.", dyaw)
            self._mover.execute_spin(dyaw,
                                     max_velocity = spin_velocity,
                                     stop_function=self.is_stop_requested)
            if self.exit_check(): return

        # drive to goal using vfh
        yaw, distance = util.get_robot_strafe(self._tf, self._target_point_odom)
        #use param velocity unless a move_velocity was included in goal
        rospy.loginfo("VFH Moving %f meters ahead.", distance)
        #start vfh and wait for one update
        self.vfh_running = True
        start_time =  rospy.Time.now()
        while not self._as.is_preempt_requested():
            rospy.sleep(0.5)
            if self.last_vfh_update > start_time: break
        #If we received a valid goal (not too short, or out of target window), then
        #the outcome should still be un-set (still ERROR).  Make the move
        if self._action_outcome == VFHMoveResult.ERROR:
            self._mover.execute_continuous_strafe(0.0,
                                                  max_velocity = move_velocity,
                                                  stop_function=self.is_stop_requested)
            self.vfh_running = False
            if self.exit_check(): return

            # turn to requested orientation
            dyaw = self.yaw_error()
            if self._orient_at_target and (np.abs(dyaw) > self._goal_orientation_tolerance):
                rospy.logdebug("VFH Rotating to goal yaw %f.", dyaw)
                self._mover.execute_spin(dyaw,
                                         spin_velocity,
                                         stop_function=self.is_stop_requested)
                if self.exit_check(): return

        rospy.logdebug("Successfully completed goal.")
        self._as.set_succeeded(result =
                               VFHMoveResult(outcome = self._action_outcome,
                                             position_error = self.position_error(),
                                             yaw_error = Float64(self.yaw_error())))
        
    def exit_check(self):
        if self._as.is_preempt_requested():
            rospy.loginfo("VFH server preempted, shutdown = %s" % self.shutdown)
            #don't attempt to send preempt result if we are in shutdown
            if not self.shutdown:
                self._as.set_preempted(result = VFHMoveResult(outcome = VFHMoveResult.EXTERNAL_STOP))
            return True
        else:
            return False
        
    def clear_check(self):
        #check for other stop requests
        if self.stop_requested(): return True
        #sector True = blocked
        self.update_sectors()
        middle_index = int(len(self.sectors)/2)
        if not self.sectors[middle_index]:
            return True
        else:
            return False
    
    def shutdown_cb(self):
        self._mover.estop()
        self.vfh_running = False
        self.shutdown = True
        #if the action server does not have an active goal, just exit
        if not self._as.is_active(): return
        #hold up the action server on shutdown, until the client requests preempt
        start_time = rospy.get_time()
        while not self._as.is_preempt_requested():
            rospy.sleep(0.05)
            elapsed = rospy.get_time() - start_time
            if  elapsed > 2.0:
                rospy.logwarn("VFH action server forced to shutdown without preempt (2 second timeout)")
                return
        self._as.set_preempted(result = VFHMoveResult(outcome = VFHMoveResult.EXTERNAL_STOP))
        start_time = rospy.get_time()
        #wait for mover to stop, then allow the rest of shutdown
        while self._mover.is_running():
            rospy.sleep(0.05)
            elapsed = rospy.get_time() - start_time
            if  elapsed > 2.0:
                rospy.logwarn("VFH action server forced to shutdown with simple_motion running (2 second timeout)")
                return
    
    def vfh_planner(self, event):
        if not self.vfh_running :
            return
        
        start_time = rospy.get_time()

        #Update the sector states, and return the inverse costs
        inverse_cost,
        target_index,
        dist_to_target,
        dist_off_course = self.update_sectors()
        
        #if we are within stop_distance initiate stop
        if distance_to_target < self.stop_distance:
            #rospy.loginfo("VFH distance_to_target < stop_distance (%f)" % self.stop_distance)
            return self.stop_with_outcome(VFHMoveResult.COMPLETE)

        if (dist_off_course > 5.0):
            return self.stop_with_outcome(VFHMoveResult.OFF_COURSE)           
                    
        #if the target is not in the active window, we are probably close to it,
        #but locally blocked: stop!
        if not 0 <= target_index < len(self.sectors):
            return self.stop_with_outcome(VFHMoveResult.MISSED_TARGET)

        #stop the mover if the path is totally blocked
        if np.all(self.sectors):
            rospy.logdebug("VFH all blocked, stop!")
            return self.stop_with_outcome(VFHMoveResult.BLOCKED)

        #If mover has started the wheels driving (initial steering alignment finished),
        #find the sector index with the lowest cost index (inverse cost!).
        if self._mover.is_running():
            min_cost_index = np.argmax(inverse_cost)
            if self.current_sector_index != min_cost_index:
                self._mover.set_strafe_angle( (min_cost_index - self.zero_offset) * self.sector_angle)
                self.current_sector_index = min_cost_index
        
        #record the time        
        self.last_vfh_update = rospy.Time.now()

        #START DEBUG CRAP
        if self.publish_debug:
            
            robot_position = self._mover.current_position
            robot_yaw = self._mover.current_yaw
            
            rospy.logdebug("TARGET SECTOR index: %d" % (target_index))
            rospy.logdebug("SELECTED SECTOR index: %d" %(self.current_sector_index))
            rospy.logdebug("SECTORS: %s" % (self.sectors))
            rospy.logdebug("INVERSE_COSTS: %s" % (inverse_cost))
            
            vfh_debug_array = []
            vfh_debug_marker = vis_msg.Marker()
            vfh_debug_marker.pose = Pose()
            vfh_debug_marker.header = std_msg.Header(0, rospy.Time(0), self.odometry_frame)
            vfh_debug_marker.type = vis_msg.Marker.ARROW
            vfh_debug_marker.color = std_msg.ColorRGBA(0, 0, 0, 1)
            vfh_debug_marker.scale = geometry_msg.Vector3(min(self.max_obstacle_distance, (distance_to_target + self._goal_obstacle_radius)), .02, .02)
            vfh_debug_marker.lifetime = rospy.Duration(0.2)
            
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
                
            #debug_marker = deepcopy(vfh_debug_marker)
            #target_yaw_odom = robot_yaw + yaw_to_target
            #quat = geometry_msg.Quaternion(\
            #       *tf.transformations.quaternion_from_euler(0, 0, target_yaw_odom))
            #debug_marker.pose.orientation = quat
            #debug_marker.pose.position = geometry_msg.Point(robot_position[0],
            #                                                robot_position[1], 0)
            #debug_marker.color = std_msg.ColorRGBA(1.0, 0.1, 1.0, 1)                
            #debug_marker.id = self.marker_id
            #self.marker_id += 1
            #vfh_debug_array.append(debug_marker)    
 
            vfh_debug_msg = vis_msg.MarkerArray(vfh_debug_array)
            self.debug_marker_pub.publish(vfh_debug_msg)
        
            #rospy.loginfo("VFH LOOP took: %.4f seconds to complete" % (rospy.get_time() - start_time))        

    def update_sectors(self):

        #get current position from mover object
        robot_position = self._mover.current_position
        robot_yaw = self._mover.current_yaw

        yaw_to_target, distance_to_target = util.get_robot_strafe(self._tf, self._target_point_odom)
        target_index = int(round(yaw_to_target/self.sector_angle) + self.zero_offset)

        #if we are too far off the line between start point, and goal, initiate stop
        dist_off_course = util.point_distance_to_line(geometry_msg.Point(*robot_position),
                                                      self._start_point,
                                                      self._target_point_odom.point)

        valid, robot_cmap_coords = self.world2map(robot_position[:2], self.costmap_info)
        
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
            #if we are close to the target, truncate the check distance
            end = self.bresenham_point(robot_cmap_coords,
                                        min(self.max_obstacle_distance,
                                           (distance_to_target+self._goal_obstacle_radius)),                                   
                                        sector_yaw,
                                        self.costmap_info.resolution)            
            sector_line = bresenham.points(start, end)

            for coords in sector_line:
                cell_value = self.costmap[tuple(coords)]

                position = self.costmap_info.resolution * (coords - robot_cmap_coords)
                distance = np.linalg.norm(position)
            
                #m is the obstacle vector magnitude
                m = cell_value * (self.max_obstacle_distance - distance)
                
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

        return inverse_cost, target_index, dist_to_target, dist_off_course


    def stop_with_outcome(self, outcome):
        self._action_outcome =  outcome
        self._mover.stop()
        self.last_vfh_update = rospy.Time.now()
        self.vfh_running = False
        return

    def handle_costmap(self, costmap):
               
        self.last_costmap_time = rospy.Time.now()
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
        target_yaw = request.yaw
        valid, robot_cmap_coords = self.world2map(robot_position[:2], self.costmap_info)
        
        ll = self.bresenham_point(robot_cmap_coords,
                                  request.width/2,
                                  (target_yaw - np.pi/2),
                                  self.costmap_info.resolution)
        ul = self.bresenham_point(robot_cmap_coords,   
                                  request.width/2,
                                  (target_yaw + np.pi/2),
                                  self.costmap_info.resolution)
        lr = self.bresenham_point(ll[0], request.distance, target_yaw, self.costmap_info.resolution)
        ur = self.bresenham_point(ul[0], request.distance, target_yaw, self.costmap_info.resolution)

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
            line_vals = self.costmap[line[:,0], line[:,1]]
            max_val = (np.amax(line_vals))
            if np.any(line_vals > self._lethal_threshold):
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

    def publish_target_marker(self, target_point):

        debug_marker = vis_msg.Marker()
        debug_marker.header = std_msg.Header(0, rospy.Time(0), self.odometry_frame)
        debug_marker.type = vis_msg.Marker.CYLINDER
        debug_marker.color = std_msg.ColorRGBA(1, 0.25, 0, 0.5)
        debug_marker.scale = geometry_msg.Vector3(.3, .3, .3)
        debug_marker.pose = Pose()
        debug_marker.pose.position = self.target_point
        debug_marker.pose.orientation = geometry_msg.Quaternion(0,0,0,1)
        debug_marker.lifetime = rospy.Duration(0) #0 is forever
        debug_marker.id = self.marker_id
        self.marker_id += 1

        vfh_debug_msg = vis_msg.MarkerArray([debug_marker])
        self.debug_marker_pub.publish(vfh_debug_msg)

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

