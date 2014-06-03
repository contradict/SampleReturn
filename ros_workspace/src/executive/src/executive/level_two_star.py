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
        self.move_base = actionlib.SimpleActionClient("planner_move_base",
                                                       move_base_msg.MoveBaseAction)
        self.CAN_interface = util.CANInterface()
       
        #get a simple_mover, it's parameters are inside a rosparam tag for this node
        self.simple_mover = simple_motion.SimpleMover('~simple_motion_params/', self.tf_listener)
        
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            if self.move_base.wait_for_server(rospy.Duration(0.1)):
                break #all services up, exit this loop
            if (rospy.get_time() - start_time) > 10.0:
                start_time =  rospy.get_time()
                self.announcer.say("Move base not available")
                rospy.logwarn('Timeout waiting for move base')
            rospy.sleep(0.1)
        
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
    
        with self.state_machine:
            
            smach.StateMachine.add('START_LEVEL_TWO',
                                   StartLeveLTwo(input_keys=['line_yaw',
                                                             'line_plan_step',
                                                             'executive_frame'],
                                                 output_keys=['action_result',
                                                              'next_line_pose'],
                                                 outcomes=['next']),
                                   transitions = {'next':'ANNOUNCE_LEVEL_TWO'})
            
            smach.StateMachine.add('ANNOUNCE_LEVEL_TWO',
                                   AnnounceState(self.announcer,
                                                 'Enter ing level two mode'),
                                   transitions = {'next':'SELECT_PLANNER'})
            
            smach.StateMachine.add('SELECT_PLANNER',
                                    SelectMotionMode(self.CAN_interface,
                                                     MODE_PLANNER),
                                    transitions = {'next':'SEARCH_LINE',
                                                  'failed':'LEVEL_TWO_ABORTED'})    

            smach.StateMachine.add('SEARCH_LINE',
                                   SearchLineManager(self.tf_listener,
                                                     self.simple_mover,
                                                     self.announcer),
                                   transitions = {'sample_detected':'PURSUE_SAMPLE',
                                                  'line_blocked':'CHOOSE_NEW_LINE',
                                                  'return_home':'ANNOUNCE_RETURN_HOME',
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
                                   transitions = {'next':'RETURN_TO_SEARCH_LINE'})   

            smach.StateMachine.add('RETURN_TO_SEARCH_LINE',
                                   DriveToPoseState(self.move_base,
                                                    self.tf_listener),
                                   transitions = {'complete':'SEARCH_LINE',
                                                  'timeout':'ANNOUNCE_RETURN_TO_SEARCH',
                                                  'sample_detected':'PURSUE_SAMPLE',
                                                  'preempted':'LEVEL_TWO_PREEMPTED'},
                                   remapping = {'target_pose':'last_line_pose',
                                                'velocity':'line_velocity',
                                                'pursue_samples':'true',
                                                'stop_on_sample':'false'})
            
            smach.StateMachine.add('CHOOSE_NEW_LINE',
                                   ChooseNewLine(self.tf_listener, self.simple_mover),
                                   transitions = {'next':'SEARCH_LINE',
                                                  'preempted':'LEVEL_TWO_PREEMPTED',
                                                  'aborted':'LEVEL_TWO_ABORTED'})
            
            smach.StateMachine.add('ROTATE_TO_NEW_LINE',
                                   DriveToPoseState(self.move_base,
                                                    self.tf_listener),
                                   transitions = {'complete':'SEARCH_LINE',
                                                  'timeout':'CHOOSE_NEW_LINE',
                                                  'sample_detected':'LEVEL_TWO_ABORTED',
                                                  'preempted':'LEVEL_TWO_PREEMPTED'},
                                   remapping = {'target_pose':'rotate_pose',
                                                'velocity':'line_velocity',
                                                'pursue_samples':'false'})

            smach.StateMachine.add('ANNOUNCE_RETURN_HOME',
                                   AnnounceState(self.announcer,
                                                 'Moving to beacon approach point'),
                                   transitions = {'next':'DRIVE_TO_BEACON_APPROACH_START'})           
            
            smach.StateMachine.add('DRIVE_TO_BEACON_APPROACH_START',
                                   DriveToPoseState(self.move_base,
                                                    self.tf_listener),
                                   transitions = {'complete':'ANNOUNCE_SEARCH_FOR_BEACON',
                                                  'timeout':'ANNOUNCE_RETURN_HOME',
                                                  'sample_detected':'ANNOUNCE_APPROACH_BEACON'},
                                   remapping = {'target_pose':'beacon_approach_pose',
                                                'velocity':'line_velocity',
                                                'detected_sample':'beacon_point',
                                                'pursue_samples':'true',
                                                'stop_on_sample':'false'})

            smach.StateMachine.add('ANNOUNCE_APPROACH_BEACON',
                                   AnnounceState(self.announcer,
                                                 'Beacon in view approach ing'),
                                   transitions = {'next':'APPROACH_BEACON'})               

            self.approach_beacon = GetPursueDetectedPointState(self.move_base,
                                                               self.tf_listener)
            
            smach.StateMachine.add('APPROACH_BEACON',
                                   self.approach_beacon,
                                   transitions = {'min_distance':'ANNOUNCE_MOUNT_PLATFORM',
                                                  'point_lost':'ANNOUNCE_RETURN_HOME',
                                                  'complete':'ANNOUNCE_RETURN_HOME',
                                                  'timeout':'ANNOUNCE_RETURN_HOME',
                                                  'aborted':'LEVEL_TWO_ABORTED'},
                                   remapping = {'velocity':'line_velocity',
                                                'pursuit_point':'beacon_point',
                                                'pursue_samples':'false',
                                                'stop_on_sample':'false'})                

            smach.StateMachine.add('ANNOUNCE_MOUNT_PLATFORM',
                                   AnnounceState(self.announcer,
                                                 'Mount ing platform'),
                                   transitions = {'next':'MOUNT_PLATFORM'})  

            smach.StateMachine.add('MOUNT_PLATFORM',
                                   DriveToPoseState(self.move_base,
                                                    self.tf_listener),
                                   transitions = {'complete':'DESELECT_PLANNER',
                                                  'timeout':'ANNOUNCE_RETURN_HOME',
                                                  'sample_detected':'LEVEL_TWO_ABORTED'},
                                   remapping = {'velocity':'line_velocity',
                                                'pursue_samples':'false',
                                                'stop_on_sample':'false'})
            
            #ADD BEACON SEARCH!
            smach.StateMachine.add('ANNOUNCE_SEARCH_FOR_BEACON',
                                   AnnounceState(self.announcer,
                                                 'Beacon not in view, searching'),
                                   transitions = {'next':'LEVEL_TWO_ABORTED'})  
            

            MODE_ENABLE = platform_srv.SelectMotionModeRequest.MODE_ENABLE
            smach.StateMachine.add('DESELECT_PLANNER',
                                    SelectMotionMode(self.CAN_interface,
                                                     MODE_ENABLE),
                                    transitions = {'next':'complete',
                                                  'failed':'LEVEL_TWO_ABORTED'})   
    
            smach.StateMachine.add('LEVEL_TWO_PREEMPTED',
                                  LevelTwoPreempted(),
                                   transitions = {'complete':'preempted',
                                                  'fail':'aborted'})
            
            smach.StateMachine.add('LEVEL_TWO_ABORTED',
                                   LevelTwoAborted(),
                                   transitions = {'recover':'ANNOUNCE_RETURN_HOME',
                                                  'fail':'aborted'})
            
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
        self.approach_beacon.userdata.paused = msg.data

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
        
        header = std_msg.Header(0, rospy.Time(0), userdata.executive_frame)        
        first_pose = geometry_msg.Pose()
        first_pose.position = geometry_msg.Point(userdata.line_plan_step, 0, 0)
        quat_array = tf.transformations.quaternion_from_euler(0, 0,  userdata.line_yaw)           
        first_pose.orientation = geometry_msg.Quaternion(*quat_array)
        userdata.next_line_pose = geometry_msg.PoseStamped(header, first_pose)        

        return 'next'

#drive to detected sample location        
class SearchLineManager(smach.State):
    def __init__(self, listener, mover, announcer):
        smach.State.__init__(self,
                             input_keys = ['next_line_pose',
                                           'line_plan_step',
                                           'line_yaw',
                                           'line_replan_distance',
                                           'blocked_check_distance',
                                           'blocked_check_width',
                                           'return_time',
                                           'detected_sample'],
                             output_keys = ['next_line_pose'],
                             outcomes=['sample_detected',
                                       'line_blocked',
                                       'return_home',
                                       'preempted', 'aborted'])
        
        self.listener = listener
        self.mover = mover
        self.announcer = announcer
                
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
        
    def execute(self, userdata):
    
        self.is_running = True
        self.last_line_blocked = False
        self.line_blocked = False
        
        self.line_yaw = userdata.line_yaw
        actual_yaw = util.get_current_robot_yaw(self.listener)
        rospy.loginfo("SEARCH LINE MANAGER, entering with yaw: " + str(np.degrees(actual_yaw)))
        
        self.yaws['left']['angle'] = actual_yaw + self.strafe_angle
        self.yaws['center']['angle'] = actual_yaw
        self.yaws['right']['angle'] = actual_yaw - self.strafe_angle
        
        self.offset_count = 0
        self.announcer.say("Start ing search line, Yaw " + str(int(math.degrees(actual_yaw))))
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
        return outcome
    
    def strafe_right(self):
        self.offset_count -= 1
        self.active_yaw = 'right'
        self.mover.execute_strafe(-self.strafe_angle, self.strafe_offset)   
    
    def strafe_left(self):
        self.offset_count += 1
        self.active_yaw = 'left'
        self.mover.execute_strafe(self.strafe_angle, self.strafe_offset)    
        
    def costmap_update(self, costmap):
        
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
                    
        return False
    
    #returns array of array for bresenham implementation    
    def check_point(self, start, distance, angle, res):
        x = np.trunc(start[0] + (distance * math.cos(angle))/res).astype('i2')
        y = np.trunc(start[1] + (distance * math.sin(angle))/res).astype('i2')
        return np.array([[x, y]])
           
class ChooseNewLine(smach.State):

    def __init__(self, tf_listener, mover):
        smach.State.__init__(self,
                             outcomes=['next', 'preempted', 'aborted'],
                             input_keys=['line_yaw',
                                         'last_line_pose',
                                         'line_plan_step',
                                         'blocked_rotation_min',
                                         'blocked_rotation_max'],
                             output_keys=['line_yaw',
                                          'next_line_pose',
                                          'rotate_pose']),  
  
        self.tf_listener = tf_listener
        self.mover = mover
    
    def execute(self, userdata):
        current_pose = util.get_current_robot_pose(self.tf_listener)
        yaw_quat = tf.transformations.quaternion_from_euler(0, 0, userdata.line_yaw)
        #stupid planner may not have the robot oriented along the search line,
        #set orientation to that value anyway
        current_pose.pose.orientation = geometry_msg.Quaternion(*yaw_quat)
        yaw_changes = np.array(range(userdata.blocked_rotation_min,
                                     userdata.blocked_rotation_max,
                                     10))
        yaw_changes = np.radians(yaw_changes)                        
        yaw_changes = np.r_[yaw_changes, -yaw_changes]
        yaw_change = random.choice(yaw_changes)
        line_yaw = userdata.line_yaw + yaw_change
        rotate_pose = util.pose_rotate(current_pose, yaw_change)
        next_line_pose = util.pose_translate_by_yaw(rotate_pose,
                                                    userdata.line_plan_step,
                                                    line_yaw)
        userdata.line_yaw = line_yaw
        userdata.rotate_pose = rotate_pose
        userdata.next_line_pose = next_line_pose
        
        self.mover.execute_spin(np.pi/2, max_velocity=0.4, acceleration=.2)
        self.mover.execute_spin(np.pi/2, max_velocity=0.4, acceleration=.2)
        
        return 'next'
  
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
        smach.State.__init__(self, outcomes=['complete','fail'])
        
    def execute(self, userdata):
        
        return 'complete'

class LevelTwoAborted(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['recover','fail'])
        
    def execute(self, userdata):
        
        return 'fail'