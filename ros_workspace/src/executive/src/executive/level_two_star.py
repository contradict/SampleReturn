#!/usr/bin/env python
import math
import collections
import threading
import random
import numpy as np

import smach
import smach_ros
import rospy
import actionlib
import tf

import std_msgs.msg as std_msg
import nav_msgs.msg as nav_msg
import visual_servo_msgs.msg as visual_servo_msg
import samplereturn_msgs.msg as samplereturn_msg
import move_base_msgs.msg as move_base_msg
import geometry_msgs.msg as geometry_msg
import platform_motion_msgs.msg as platform_msg
import platform_motion_msgs.srv as platform_srv

import samplereturn.simple_motion as simple_motion

from executive.executive_states import DriveToPoseState
from executive.executive_states import PursueDetectedPoint
from executive.executive_states import SelectMotionMode
from executive.executive_states import AnnounceState
from executive.executive_states import GetPursueDetectedPointState
from executive.executive_states import GetSimpleMoveState

import samplereturn.util as util
import samplereturn.bresenham as bresenham

class LevelTwoStar(object):
    
    def __init__(self):
        
        rospy.on_shutdown(self.shutdown_cb)
        self.node_params = util.get_node_params()
        self.tf_listener = tf.TransformListener()
        self.executive_frame = self.node_params.executive_frame
        self.beacon_approach_point = self.node_params.beacon_approach_point
        
        #calculate beacon approach pose now, in /map
        header = std_msg.Header(0, rospy.Time(0), '/map')
        point = geometry_msg.Point(self.beacon_approach_point['x'],
                                   self.beacon_approach_point['y'], 0)
        quat_array = tf.transformations.quaternion_from_euler(0, 0, math.pi)           
        pose = geometry_msg.Pose(point, geometry_msg.Quaternion(*quat_array))
        self.beacon_approach_pose = geometry_msg.PoseStamped(header, pose)
        
        #interfaces
        self.announcer = util.AnnouncerInterface("audio_navigate")
        self.CAN_interface = util.CANInterface()
       
        #get a simple_mover, it's parameters are inside a rosparam tag for this node
        self.simple_mover = simple_motion.SimpleMover('~simple_motion_params/', self.tf_listener)
        
        self.state_machine = smach.StateMachine(
                outcomes=['complete', 'preempted', 'aborted'],
                input_keys = ['action_goal'],
                output_keys = ['action_result'])
    
        #these are important values!  master frame id and return timing
        self.state_machine.userdata.executive_frame = self.executive_frame
        self.state_machine.userdata.start_time = rospy.Time.now()
        self.state_machine.userdata.return_time = rospy.Time.now() + \
                                                  rospy.Duration(self.node_params.return_time_minutes*60)
        self.state_machine.userdata.pre_cached_id = samplereturn_msg.NamedPoint.PRE_CACHED
        
        #beacon approach
        self.state_machine.userdata.max_pursuit_error = self.node_params.max_pursuit_error       
        self.state_machine.userdata.min_pursuit_distance = self.node_params.min_pursuit_distance
        self.state_machine.userdata.max_point_lost_time = self.node_params.max_point_lost_time
        self.state_machine.userdata.beacon_approach_pose = self.beacon_approach_pose
        
        #search line parameters
        self.state_machine.userdata.line_plan_step = self.node_params.line_plan_step
        self.state_machine.userdata.line_replan_distance = self.node_params.line_replan_distance
        self.state_machine.userdata.line_velocity = self.node_params.line_velocity
        self.state_machine.userdata.blocked_check_distance = self.node_params.blocked_check_distance
        self.state_machine.userdata.blocked_check_width = self.node_params.blocked_check_width        
        self.state_machine.userdata.blocked_rotation_min = self.node_params.blocked_rotation_min
        self.state_machine.userdata.blocked_rotation_max = self.node_params.blocked_rotation_max
        self.state_machine.userdata.motion_check_interval = self.node_params.motion_check_interval
        self.state_machine.userdata.min_motion = self.node_params.min_motion

        #search line variables
        self.state_machine.userdata.next_line_pose = None
        self.state_machine.userdata.last_line_pose = None
        self.state_machine.userdata.line_yaw = 0 #IN RADIANS!

        #subscriber controlled userdata
        self.state_machine.userdata.paused = False        
        self.state_machine.userdata.detected_sample = None
        self.state_machine.userdata.beacon_point = None
        
        #use these as booleans in remaps
        self.state_machine.userdata.true = True
        self.state_machine.userdata.false = False
    
        #motion mode stuff
        planner_mode = self.node_params.planner_mode
        MODE_PLANNER = getattr(platform_srv.SelectMotionModeRequest, planner_mode)
        MODE_SERVO = platform_srv.SelectMotionModeRequest.MODE_SERVO
    
        with self.state_machine:
            
            smach.StateMachine.add('START_LEVEL_TWO',
                                   StartLeveLTwo(input_keys=[],
                                                 output_keys=['action_result',
                                                              'outbound'],
                                                 outcomes=['next']),
                                   transitions = {'next':'ANNOUNCE_LEVEL_TWO'})
            
            smach.StateMachine.add('ANNOUNCE_LEVEL_TWO',
                                   AnnounceState(self.announcer,
                                                 'Enter ing level two mode'),
                                   transitions = {'next':'SELECT_SERVO'})
            
            smach.StateMachine.add('SELECT_SERVO',
                                    SelectMotionMode(self.CAN_interface,
                                                     MODE_SERVO),
                                    transitions = {'next':'STAR_MANAGER',
                                                  'failed':'LEVEL_TWO_ABORTED'})
            
            smach.StateMachine.add('STAR_MANAGER',
                                   StarManager(self.tf_listener,
                                               self.announcer),
                                   transitions = {'start_line':'LINE_MANAGER',
                                                  'rotate':'ROTATION_MANAGER',
                                                  'return_home':'ANNOUNCE_RETURN_HOME'})

            smach.StateMachine.add('LINE_MANAGER',
                                   SearchLineManager(self.tf_listener,
                                                     self.simple_mover,
                                                     self.announcer,
                                                     self.executive_frame),
                                   transitions = {'sample_detected':'PURSUE_SAMPLE',
                                                  'line_blocked':'STAR_MANAGER',
                                                  'next_spoke':'STAR_MANAGER',
                                                  'return_home':'STAR_MANAGER',
                                                  'preempted':'LEVEL_TWO_PREEMPTED',
                                                  'aborted':'LEVEL_TWO_ABORTED'})
            
            smach.StateMachine.add('ROTATION_MANAGER',
                                   RotationManager(self.tf_listener, self.simple_mover),
                                   transitions = {'next':'LINE_MANAGER',
                                                  'preempted':'LEVEL_TWO_PREEMPTED',
                                                  'aborted':'LEVEL_TWO_ABORTED'})
            
            @smach.cb_interface(input_keys=['detected_sample'])
            def get_pursuit_goal_cb(userdata, request):
                goal = samplereturn_msg.GeneralExecutiveGoal()
                goal.input_point = userdata.detected_sample
                goal.input_string = "level_two_pursuit_request"
                return goal
                            
            smach.StateMachine.add('PURSUE_SAMPLE',
                                  smach_ros.SimpleActionState('pursue_sample',
                                  samplereturn_msg.GeneralExecutiveAction,
                                  goal_cb = get_pursuit_goal_cb),
                                  transitions = {'succeeded':'ANNOUNCE_RETURN_TO_SEARCH',
                                                 'aborted':'ANNOUNCE_RETURN_TO_SEARCH'})

            smach.StateMachine.add('ANNOUNCE_RETURN_TO_SEARCH',
                                   AnnounceState(self.announcer,
                                                 'Return ing to search line'),
                                   transitions = {'next':'STAR_MANAGER'})   

            smach.StateMachine.add('ANNOUNCE_RETURN_HOME',
                                   AnnounceState(self.announcer,
                                                 'Moving to beacon approach point'),
                                   transitions = {'next':'ANNOUNCE_APPROACH_BEACON'})           

            smach.StateMachine.add('ANNOUNCE_APPROACH_BEACON',
                                   AnnounceState(self.announcer,
                                                 'Beacon in view approach ing'),
                                   transitions = {'next':'ANNOUNCE_MOUNT_PLATFORM'})               

            smach.StateMachine.add('ANNOUNCE_MOUNT_PLATFORM',
                                   AnnounceState(self.announcer,
                                                 'Mount ing platform'),
                                   transitions = {'next':'MOUNT_PLATFORM'})  

            self.mount_platform = GetSimpleMoveState(self.simple_mover, self.tf_listener)
            
            smach.StateMachine.add('MOUNT_PLATFORM',
                                   self.mount_platform,
                                   transitions = {'complete':'DESELECT_SERVO',
                                                  'timeout':'ANNOUNCE_RETURN_HOME',
                                                  'sample_detected':'LEVEL_TWO_ABORTED',
                                                  'preempted':'LEVEL_TWO_PREEMPTED',
                                                  'aborted':'LEVEL_TWO_ABORTED'},
                                   remapping = {'pursue_samples':'true'})

            #ADD BEACON SEARCH!
            smach.StateMachine.add('ANNOUNCE_SEARCH_FOR_BEACON',
                                   AnnounceState(self.announcer,
                                                 'Beacon not in view, searching'),
                                   transitions = {'next':'LEVEL_TWO_ABORTED'})  
            

            MODE_ENABLE = platform_srv.SelectMotionModeRequest.MODE_ENABLE
            smach.StateMachine.add('DESELECT_SERVO',
                                    SelectMotionMode(self.CAN_interface,
                                                     MODE_ENABLE),
                                    transitions = {'next':'complete',
                                                  'failed':'LEVEL_TWO_ABORTED'})   
    
            smach.StateMachine.add('LEVEL_TWO_PREEMPTED',
                                  LevelTwoPreempted(),
                                   transitions = {'next':'preempted'})
            
            smach.StateMachine.add('LEVEL_TWO_ABORTED',
                                   LevelTwoAborted(),
                                   transitions = {'next':'aborted'})
            
        #action server wrapper    
        level_two_server = smach_ros.ActionServerWrapper(
            'level_two', samplereturn_msg.GeneralExecutiveAction,
            wrapped_container = self.state_machine,
            succeeded_outcomes = ['complete'],
            preempted_outcomes = ['preempted'],
            aborted_outcomes = ['aborted'],
            goal_key = 'action_goal',
            result_key = 'action_result')
        
        #introspection server
        sls = smach_ros.IntrospectionServer('smach_grab_introspection',
                                            self.state_machine,
                                            '/START_LEVEL_TWO')

        rospy.Subscriber('detected_sample_search',
                        samplereturn_msg.NamedPoint,
                        self.sample_update)
        
        rospy.Subscriber("beacon_pose",
                        geometry_msg.PoseStamped,
                        self.beacon_update)
        
        rospy.Subscriber("pause_state", std_msg.Bool, self.pause_state_update)
        
        #start action servers and services
        sls.start()
        level_two_server.run_server()
        rospy.spin()
        sls.stop()

    def pause_state_update(self, msg):
        self.state_machine.userdata.paused = msg.data
        self.mount_platform.userdata.paused = msg.data

    def sample_update(self, sample):
        try:
            self.tf_listener.waitForTransform(self.executive_frame,
                                              sample.header.frame_id,
                                              sample.header.stamp,
                                              rospy.Duration(1.0))
            point_in_frame = self.tf_listener.transformPoint(self.executive_frame, sample)
            sample.point = point_in_frame.point
            self.state_machine.userdata.detected_sample = sample
        except tf.Exception:
            rospy.logwarn("LEVEL_TWO failed to transform search detection point!")        
            
    def beacon_update(self, beacon_pose):
        beacon_point = geometry_msg.PointStamped(beacon_pose.header,
                                                 beacon_pose.pose.position)
        try:
            self.tf_listener.waitForTransform(self.executive_frame,
                                              beacon_point.header.frame_id,
                                              beacon_point.header.stamp,
                                              rospy.Duration(1.0))
            point_in_frame = self.tf_listener.transformPoint(self.executive_frame, beacon_point)
            point_in_frame.point.x += 1.5 #start point is 1.5 meters in front of beacon
            beacon_point.point = point_in_frame.point
            self.state_machine.userdata.beacon_point = beacon_point
            self.approach_beacon.userdata.beacon_point = beacon_point
        except tf.Exception:
            rospy.logwarn("LEVEL_TWO failed to transform beacon detection pose!")        
 
    def shutdown_cb(self):
        self.state_machine.request_preempt()
        while self.state_machine.is_running():
            rospy.sleep(0.1)
    
#searches the globe   
class StartLeveLTwo(smach.State):

    def execute(self, userdata):
        
        result = samplereturn_msg.GeneralExecutiveResult()
        result.result_string = 'initialized'
        userdata.action_result = result
        
        userdata.outbound = False

        return 'next'

class StarManager(smach.State):
    def __init__(self, listener, announcer):
        smach.State.__init__(self,
                             input_keys = ['line_yaw',
                                           'outbound'],
                             output_keys = ['line_yaw',
                                            'outbound'],
                             outcomes=['start_line',
                                       'rotate',
                                       'return_home',
                                       'preempted', 'aborted'])
        
        self.listener = listener
        self.announcer = announcer  

        self.spokes = list(np.linspace(0, 2*np.pi, 10, endpoint=False))

    def execute(self, userdata):
        
        #are we returning in?
        if not userdata.outbound:
            userdata.line_yaw = self.spokes.pop(0)
            userdata.outbound = True
            self.announcer.say("Start ing spoke, Yaw " + str(int(math.degrees(userdata.line_yaw))))
            return 'rotate'        
        
        if userdata.outbound:
            self.announcer.say("Return ing spoke, Yaw " + str(int(math.degrees(userdata.line_yaw))))
            userdata.line_yaw = util.get_yaw_to_origin(self.listener)
            userdata.outbound = False
            return 'rotate'
        
        return 'start_line'    

class RotationManager(smach.State):

    def __init__(self, tf_listener, mover):
        smach.State.__init__(self,
                             outcomes=['next', 'preempted', 'aborted'],
                             input_keys=['line_yaw',
                                         'outbound'],
                             output_keys=['line_yaw',
                                          'outbound',
                                          'rotate_pose']),  
  
        self.tf_listener = tf_listener
        self.mover = mover
    
    def execute(self, userdata):
        actual_yaw = util.get_current_robot_yaw(self.tf_listener)
        rospy.loginfo("ROTATION MANAGER first move, actual_yaw, line_yaw: " +
                      str(int(math.degrees(actual_yaw))) + " " +
                      str(int(math.degrees(userdata.line_yaw))))
        rotate_yaw = util.unwind(userdata.line_yaw - actual_yaw)
        rospy.loginfo("ROTATION MANAGER first move, rotate_yaw: " + str(int(math.degrees(userdata.line_yaw))))
        self.mover.execute_spin(rotate_yaw, max_velocity=0.5, acceleration=.25)
        
        actual_yaw = util.get_current_robot_yaw(self.tf_listener)
        rotate_yaw = util.unwind(userdata.line_yaw - actual_yaw)
        rospy.loginfo("ROTATION MANAGER second move, actual_yaw, line_yaw: " +
                      str(int(math.degrees(actual_yaw))) + " " +
                      str(int(math.degrees(userdata.line_yaw))))
        rotate_yaw = util.unwind(userdata.line_yaw - actual_yaw)
        rospy.loginfo("ROTATION MANAGER second move, rotate_yaw: " + str(int(math.degrees(userdata.line_yaw))))
        self.mover.execute_spin(rotate_yaw, max_velocity=0.05, acceleration=.025)
                
        return 'next'

#drive to detected sample location        
class SearchLineManager(smach.State):
    def __init__(self, listener, mover, announcer, executive_frame):
        smach.State.__init__(self,
                             input_keys = ['line_plan_step',
                                           'line_yaw',
                                           'outbound',
                                           'blocked_check_distance',
                                           'blocked_check_width',
                                           'return_time',
                                           'detected_sample',
                                           'executive_frame'],
                             output_keys = ['next_line_pose',
                                            'detected_sample'],
                             outcomes=['sample_detected',
                                       'line_blocked',
                                       'next_spoke',
                                       'return_home',
                                       'preempted', 'aborted'])
        
        self.listener = listener
        self.mover = mover
        self.announcer = announcer
        self.executive_frame = executive_frame
                
        self.costmap_listener = rospy.Subscriber('local_costmap',
                                        nav_msg.OccupancyGrid,
                                        self.costmap_update)
        self.costmap = nav_msg.OccupancyGrid()
        self.new_costmap_available = True
        
        self.is_running = False
        self.line_yaw = 0
        self.active_yaw = ''
        self.strafe_angle = np.pi/4
        self.strafe_offset = 4.0
        self.offset_count_limit = 1
 
        self.yaws = {'left':{'angle':0, 'blocked':False},
                    'center':{'angle':0, 'blocked':False},
                    'right':{'angle':0, 'blocked':False}}
        
        self.debug_map_pub = rospy.Publisher('/test_costmap', nav_msg.OccupancyGrid)
        
        self.detected_sample = None
        rospy.Subscriber('detected_sample_search',
                        samplereturn_msg.NamedPoint,
                        self.sample_update)
        
    def execute(self, userdata):
    
        self.is_running = True
        self.detected_sample = None
        
        self.line_yaw = userdata.line_yaw
        actual_yaw = util.get_current_robot_yaw(self.listener)
        rospy.loginfo("SEARCH LINE MANAGER, entering with actual_yaw: " + str(np.degrees(actual_yaw)))
        rospy.loginfo("SEARCH LINE MANAGER, entering with line_yaw: " + str(np.degrees(self.line_yaw)))
        
        self.yaws['left']['angle'] = actual_yaw + self.strafe_angle
        self.yaws['center']['angle'] = actual_yaw
        self.yaws['right']['angle'] = actual_yaw - self.strafe_angle
        
        if userdata.outbound:
            self.offset_count_limit = 1
        else:
            self.offset_count_limit = 2
        
        self.offset_count = 0
        
        #wait for costmaps to update
        rospy.sleep(3.0)
        
        #giant stupid case loop
        while not rospy.is_shutdown():  

            if rospy.Time.now() > userdata.return_time:
                self.announcer.say("Search time expired")
                self.active_yaw = ''
                return self.return_outcome('return_home')
            
            #check preempt after deciding whether or not to calculate next line pose
            if self.preempt_requested():
                rospy.loginfo("PREEMPT REQUESTED IN LINE MANAGER")
                self.service_preempt()
                return self.return_outcome('preempted')
            
            if self.detected_sample is not None:
                return self.return_outcome("sample_detected")
 
            current_pose = util.get_current_robot_pose(self.listener)
            origin_distance = np.sqrt(current_pose.pose.position.x**2 + current_pose.pose.position.y**2)
            if not userdata.outbound and origin_distance < 10:
                return self.return_outcome('next_spoke')        
                   
            #first check if we are offset from line
            if (self.offset_count > 0) and not self.yaws['right']['blocked']:
                #left of line
                self.announcer.say('Right clear, strafe ing to line')
                self.strafe_right()
                continue
            elif (self.offset_count < 0) and not self.yaws['left']['blocked']:
                #right of line
                self.announcer.say('Left clear, strafe ing to line')
                self.strafe_left()
                continue
            
            #if not offset or returning to line is blocked, going straight is next priority
            if not self.yaws['center']['blocked']:
                self.announcer.say("Line clear, continue ing")    
                self.active_yaw = 'center'
                remaining = self.mover.execute_strafe(0, userdata.line_plan_step)
                continue         
            
            #if going straight and returning to the line are not possible,             
            under_offset_limit = np.abs(self.offset_count) < self.offset_count_limit
            if not self.yaws['left']['blocked'] and under_offset_limit:
                self.announcer.say("Obstacle in line, strafe ing left")
                self.strafe_left()
                continue
            elif not self.yaws['right']['blocked'] and under_offset_limit:
                self.announcer.say("Obstacle in line, strafe ing right")
                self.strafe_right()
                continue
            else:             
                #lethal cells in all three yaw check lines
                self.announcer.say("Line is blocked, rotate ing")
                self.active_yaw = ''
                return self.return_outcome('line_blocked')               
                
                #finished or stopped in offset move
                if self.yaws[self.active_yaw]['blocked']:
                    self.active_yaw = ''
                    return self.return_outcome('line_blocked')
       
            rospy.sleep(0.1)
        
        return 'aborted'
    
    def return_outcome(self, outcome):
        #set active yaw to sorta none out of here
        self.active_yaw = ''
        self.is_running = False
        return outcome
    
    def strafe_right(self):
        self.offset_count -= 1
        self.active_yaw = 'right'
        rospy.loginfo("STRAFING RIGHT to offset: %s" % (self.strafe_offset))
        self.mover.execute_strafe(-self.strafe_angle, self.strafe_offset)   
    
    def strafe_left(self):
        self.offset_count += 1
        self.active_yaw = 'left'
        rospy.loginfo("STRAFING LEFT to offset: %s" % (self.strafe_offset))
        self.mover.execute_strafe(self.strafe_angle, self.strafe_offset)    

    def sample_update(self, sample):
        try:
            self.listener.waitForTransform('/odom',
                                              sample.header.frame_id,
                                              sample.header.stamp,
                                              rospy.Duration(1.0))
            point_in_frame = self.listener.transformPoint(self.executive_frame, sample)
            sample.point = point_in_frame.point
            self.detected_sample = sample
            self.mover.stop()    
        except tf.Exception:
            rospy.logwarn("LEVEL_TWO failed to transform search detection point!")  
        
    def costmap_update(self, costmap):
        
        if not self.is_running:
            return
        
        if self.debug_map_pub.get_num_connections() > 0:
            publish_debug = True
        else:
            publish_debug = False
        
        lethal_threshold = 90
        check_width = 1.1
        check_dist = 6
        resolution = costmap.info.resolution
        origin = (np.trunc(costmap.info.width/2),
                  np.trunc(costmap.info.height/2))

        map_np = np.array(costmap.data, dtype='i1').reshape((costmap.info.height,costmap.info.width))
        
        blocked_count = 0
        total_count = 0
        
        #rospy.loginfo("LINE MANAGER yaws: " + str(self.yaws))
                
        for name, yaw in self.yaws.iteritems():
            
            angle = yaw['angle']
            
            ll = self.check_point(origin, check_width, (angle - math.pi/2), resolution)
            ul = self.check_point(origin, check_width, (angle + math.pi/2), resolution)
            lr = self.check_point(ll[0], check_dist, angle, resolution)
            ur = self.check_point(ul[0], check_dist, angle, resolution)
            
            start_points = bresenham.points(ll, ul)
            end_points = bresenham.points(lr, ur)
            total_count += len(start_points)
            
            #check lines for lethal values
            for start, end in zip(start_points, end_points):
                line = bresenham.points(start[None,:], end[None,:])
                line_vals = map_np[line[:,1], line[:,0]]
                max_val = (np.amax(line_vals))
                if np.any(line_vals > lethal_threshold):
                    #for debug, mark lethal lines
                    if publish_debug: map_np[line[:,1], line[:,0]] = 64
                    yaw['blocked'] = True
                    if name == self.active_yaw:
                        self.mover.stop()
                    break
                yaw['blocked']=False    
                    
            #if anything is subscribing to the test map, publish it
            if publish_debug:
                map_np[start_points[:,1], start_points[:,0]] = 64
                map_np[end_points[:,1], end_points[:,0]] = 64

        if publish_debug:
            costmap.data = list(np.reshape(map_np, -1))
            self.debug_map_pub.publish(costmap)                        
                    
        return
    
    #returns array of array for bresenham implementation    
    def check_point(self, start, distance, angle, res):
        x = np.trunc(start[0] + (distance * math.cos(angle))/res).astype('i2')
        y = np.trunc(start[1] + (distance * math.sin(angle))/res).astype('i2')
        return np.array([[x, y]])
  
class StartReturnHome(smach.State):
 
    def __init__(self, announcer):

        smach.State.__init__(self,
                             outcomes=['next',
                                       'preempted',
                                       'aborted'],
                             input_keys=['beacon_approach_point'],
                             output_keys=['target_pose',
                                          'velocity',
                                          'pursue_samples',
                                          'beacon_point'])
        
        self.announcer = announcer

    def execute(self, userdata):
        
        self.announcer.say("Return ing to platform")
        
        return 'next'
    
class LevelTwoPreempted(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next'])
        
    def execute(self, userdata):
        
        return 'next'

class LevelTwoAborted(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next'])
        
    def execute(self, userdata):
        
        return 'next'