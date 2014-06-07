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
        self.beacon_approach_point = self.node_params.beacon_approach_point

        self.world_fixed_frame = rospy.get_param("world_fixed_frame", "map")
        self.odometry_frame = rospy.get_param("odometry_frame", "odom")
        
        #calculate beacon approach pose now, in /map
        header = std_msg.Header(0, rospy.Time(0), self.world_fixed_frame)
        point = geometry_msg.Point(self.beacon_approach_point['x'],
                                   self.beacon_approach_point['y'], 0)
        quat_array = tf.transformations.quaternion_from_euler(0, 0, math.pi)           
        pose = geometry_msg.Pose(point, geometry_msg.Quaternion(*quat_array))
        self.beacon_approach_pose = geometry_msg.PoseStamped(header, pose)
        
        #interfaces
        self.announcer = util.AnnouncerInterface("audio_navigate")
        self.CAN_interface = util.CANInterface()
       
        #get a simple_mover, it's parameters are inside a rosparam tag for this node
        self.simple_mover = simple_motion.SimpleMover('~simple_motion_params/',
                                                      self.tf_listener,
                                                      stop_function = self.check_for_stop)
        
        #strafe definitions, offset is length along strafe line
        #the yaws have a static angle, which the direction from base_link the robot strafes
        self.strafes = rospy.get_param('strafes')
        
        rospy.loginfo('STRAFES: %s' % (self.strafes))       
               
        self.state_machine = smach.StateMachine(
                outcomes=['complete', 'preempted', 'aborted'],
                input_keys = ['action_goal'],
                output_keys = ['action_result'])
    
        self.state_machine.userdata.strafes = self.strafes
        self.state_machine.userdata.active_strafe_key = None

        #these are important values!  master frame id and return timing
        self.state_machine.userdata.world_fixed_frame = self.world_fixed_frame
        self.state_machine.userdata.odometry_frame = self.odometry_frame
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
        self.state_machine.userdata.spin_velocity = self.node_params.spin_velocity
        self.state_machine.userdata.obstacle_check_distance = self.node_params.obstacle_check_distance
        self.state_machine.userdata.obstacle_check_width = self.node_params.obstacle_check_width        

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
                                                     self.announcer),
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
            def pursuit_goal_cb(userdata, request):
                goal = samplereturn_msg.GeneralExecutiveGoal()
                goal.input_point = userdata.detected_sample
                goal.input_string = "level_two_pursuit_request"
                return goal
            
            @smach.cb_interface(output_keys=['detected_sample'])
            def pursuit_result_cb(userdata, status, result):
                #clear samples after a pursue action
                userdata.detected_sample = None
                            
            smach.StateMachine.add('PURSUE_SAMPLE',
                                  smach_ros.SimpleActionState('pursue_sample',
                                  samplereturn_msg.GeneralExecutiveAction,
                                  goal_cb = pursuit_goal_cb,
                                  result_cb = pursuit_result_cb),
                                  transitions = {'succeeded':'ANNOUNCE_RETURN_TO_SEARCH',
                                                 'aborted':'ANNOUNCE_RETURN_TO_SEARCH'})

            smach.StateMachine.add('ANNOUNCE_RETURN_TO_SEARCH',
                                   AnnounceState(self.announcer,
                                                 'Rotate ing to search line'),
                                   transitions = {'next':'ROTATION_MANAGER'})   

            smach.StateMachine.add('ANNOUNCE_RETURN_HOME',
                                   AnnounceState(self.announcer,
                                                 'Move ing to beacon approach point'),
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
        
        rospy.Subscriber('costmap_check',
                                samplereturn_msg.CostmapCheck,
                                self.handle_costmap_check)

        
        rospy.Subscriber("pause_state", std_msg.Bool, self.pause_state_update)
        
        #start action servers and services
        sls.start()
        level_two_server.run_server()
        rospy.spin()
        sls.stop()
    
    #this is the callback from simple_mover that sees if it should stop!    
    def check_for_stop(self):
        active_strafe_key = self.state_machine.userdata.active_strafe_key
        
        if active_strafe_key is not None and self.strafes[active_strafe_key]['blocked']:
            return True
        
        if self.state_machine.userdata.detected_sample is not None:
            rospy.loginfo("LEVEL_TWO stopping simple_move on sample detection")
            return True
        
        return False

    def handle_costmap_check(self, costmap_check):
        for strafe_key, blocked in zip(costmap_check.strafe_keys,
                                       costmap_check.blocked):
            self.strafes[strafe_key]['blocked'] = blocked                        

    def pause_state_update(self, msg):
        self.state_machine.userdata.paused = msg.data
        self.mount_platform.userdata.paused = msg.data
        
    def sample_update(self, sample):
        try:
            self.tf_listener.waitForTransform(self.odometry_frame,
                                              sample.header.frame_id,
                                              sample.header.stamp,
                                              rospy.Duration(1.0))
            point_in_frame = self.tf_listener.transformPoint(self.odometry_frame, sample)
            sample.point = point_in_frame.point
            self.state_machine.userdata.detected_sample = sample
        except tf.Exception, e:
            rospy.logwarn("LEVEL_TWO failed to transform search detection point %s->%s: %s",
                    sample.header.frame_id, self.odometry_frame, e)
            
    def beacon_update(self, beacon_pose):
        beacon_point = geometry_msg.PointStamped(beacon_pose.header,
                                                 beacon_pose.pose.position)
        try:
            self.tf_listener.waitForTransform(self.odometry_frame,
                                              beacon_point.header.frame_id,
                                              beacon_point.header.stamp,
                                              rospy.Duration(1.0))
            point_in_frame = self.tf_listener.transformPoint(self.odometry_frame, beacon_point)
            point_in_frame.point.x += 1.5 #start point is 1.5 meters in front of beacon
            beacon_point.point = point_in_frame.point
            self.state_machine.userdata.beacon_point = beacon_point
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
    def __init__(self, tf_listener, announcer):
        smach.State.__init__(self,
                             input_keys = ['line_yaw',
                                           'outbound',
                                           'world_fixed_frame'],
                             output_keys = ['line_yaw',
                                            'outbound'],
                             outcomes=['start_line',
                                       'rotate',
                                       'return_home',
                                       'preempted', 'aborted'])
        
        self.tf_listener = tf_listener
        self.announcer = announcer  

        self.spokes = list(np.linspace(0, 2*np.pi, 4, endpoint=False))
        self.starting_spoke = np.radians(-82)

    def execute(self, userdata):
        

        
        #are we returning to the center?
        if not userdata.outbound:
            #for now, just kill the state machine when done
            if len(self.spokes) == 0:
                rospy.loginfo("STAR MANAGER finished inbound move, last spoke")
                return 'return_home'
            userdata.line_yaw = util.unwind(self.spokes.pop(0) + self.starting_spoke)
            userdata.outbound = True
            self.announcer.say("Start ing on spoke, Yaw " + str(int(math.degrees(userdata.line_yaw))))
            return 'rotate'        
        
        if userdata.outbound:
            self.announcer.say("Return ing on spoke, Yaw " + str(int(math.degrees(userdata.line_yaw))))
            userdata.line_yaw = util.get_yaw_to_origin(self.tf_listener,
                    userdata.world_fixed_frame)
            userdata.outbound = False
            return 'rotate'
        
        return 'start_line'    

class RotationManager(smach.State):

    def __init__(self, tf_listener, mover):
        smach.State.__init__(self,
                             outcomes=['next', 'preempted', 'aborted'],
                             input_keys=['line_yaw',
                                         'spin_velocity',
                                         'outbound',
                                         'world_fixed_frame'],
                             output_keys=['line_yaw',
                                          'outbound',
                                          'rotate_pose']),  
  
        self.tf_listener = tf_listener
        self.mover = mover
    
    def execute(self, userdata):
        actual_yaw = util.get_current_robot_yaw(self.tf_listener,
                userdata.world_fixed_frame)
        rotate_yaw = util.unwind(userdata.line_yaw - actual_yaw)
        rospy.loginfo("ROTATION MANAGER line_yaw: %.1f, actual_yaw: %.1f, rotate_yaw: %.1f" %(
                      np.degrees(userdata.line_yaw),
                      np.degrees(actual_yaw),
                      np.degrees(rotate_yaw)))
        #execute the actual spin!
        error = self.mover.execute_spin(rotate_yaw, max_velocity=userdata.spin_velocity)
        actual_yaw = util.get_current_robot_yaw(self.tf_listener,
                userdata.world_fixed_frame)
        rospy.loginfo("ROTATION MANAGER returned actual_yaw: %.1f, error: %.4f" % (
                      np.degrees(actual_yaw),
                      error))
        return 'next'

#drive to detected sample location        
class SearchLineManager(smach.State):
    def __init__(self, tf_listener, mover, announcer):
        smach.State.__init__(self,
                             input_keys = ['strafes',
                                           'line_yaw',
                                           'outbound',
                                           'obstacle_check_distance',
                                           'obstacle_check_width',
                                           'return_time',
                                           'detected_sample',
                                           'world_fixed_frame',
                                           'odometry_frame',
                                           ],
                             output_keys = ['next_line_pose',
                                            'detected_sample',
                                            'active_strafe_key'],
                             outcomes=['sample_detected',
                                       'line_blocked',
                                       'next_spoke',
                                       'return_home',
                                       'preempted', 'aborted'])
        
        self.tf_listener = tf_listener
        self.mover = mover
        self.announcer = announcer

    def execute(self, userdata):
    
        self.strafes = userdata.strafes
        userdata.detected_sample = None
        
        #the desired yaw of this line
        self.line_yaw = userdata.line_yaw
 
        #if on return line, allow a little more strafing, this will always work perfectly
        self.offset_count = 0
        self.offset_distance = 0
        if userdata.outbound:
            #reset offset count on outbound moves
            self.offset_count_limit = 1
        else:
            self.offset_count_limit = 2
        
        actual_yaw = util.get_current_robot_yaw(self.tf_listener,
                userdata.world_fixed_frame)
        current_pose = util.get_current_robot_pose(self.tf_listener,
                userdata.world_fixed_frame)
        
        rospy.loginfo("SEARCH LINE MANAGER, outbound: %s,  line_yaw: %.1f,  actual_yaw: %.1f,  position: (%.2f,%.2f) " % (
                       userdata.outbound,
                       np.degrees(userdata.line_yaw),
                       np.degrees(actual_yaw),
                       current_pose.pose.position.x,
                       current_pose.pose.position.y))
        
        #wait for costmaps to update
        rospy.sleep(3.0)
        
        #useful condition lambdas and other flags for dumb planner behaviour
        under_offset_limit = lambda: np.abs(self.offset_count) < self.offset_count_limit
        first_angle_choice = 'left' if userdata.outbound else 'right'
        second_angle_choice = 'right' if userdata.outbound else 'left'
        
        #giant stupid case loop
        while not rospy.is_shutdown():  

            if rospy.Time.now() > userdata.return_time:
                self.announcer.say("Search time expired")
                return self.with_outcome('return_home')
            
            #check preempt after deciding whether or not to calculate next line pose
            if self.preempt_requested():
                rospy.loginfo("LINE MANAGER: preempt requested")
                self.service_preempt()
                return self.with_outcome('preempted')
            
            if userdata.detected_sample is not None:
                return self.with_outcome("sample_detected")
 
            current_pose = util.get_current_robot_pose(self.tf_listener,
                    userdata.world_fixed_frame)
            origin_distance = np.sqrt(current_pose.pose.position.x**2 + current_pose.pose.position.y**2)
            if not userdata.outbound and origin_distance < 10:
                return self.with_outcome('next_spoke')        
            
            #BEGINNING OF THE MIGHTY LINE MANAGER CASE LOOP       
            #first check if we are offset from line either direction, and if so, is the path
            #back the line clear.  If is is, strafe towards the line
            if (self.offset_count > 0) and not self.strafes['right']['blocked']:
                #left of line
                self.announcer.say('Right clear, strafe ing to line')
                self.strafe('right', userdata)
            elif (self.offset_count < 0) and not self.strafes['left']['blocked']:
                #right of line
                self.announcer.say('Left clear, strafe ing to line')
                self.strafe('left', userdata)
            
            #if not offset or returning to line is blocked, going straight is next priority
            elif not self.strafes['center']['blocked']:
                self.announcer.say("Line clear, continue ing")    
                self.strafe('center', userdata)
               
            #if returning to the line not possible or necessary,
            #and going straight isn't possible: offset from line             
            elif not self.strafes[first_angle_choice]['blocked'] and under_offset_limit():
                self.announcer.say("Obstacle in line, strafe ing %s" % (first_angle_choice))
                self.strafe(first_angle_choice, userdata)
                
            elif not self.strafes[second_angle_choice]['blocked'] and under_offset_limit():
                self.announcer.say("Obstacle in line, strafe ing %s" % (second_angle_choice))
                self.strafe(second_angle_choice, userdata)
            
            #getting here means lethal cells in all three yaw check lines,
            #or that the only unblocked line takes us outside our allowable offset
            else:             
                self.announcer.say("Line is blocked, rotate ing")
                return self.with_outcome('line_blocked')               
       
            #a little delay to help make sure costmap are updated and such
            rospy.sleep(0.5)
        
        return 'aborted'
    
    def with_outcome(self, outcome):
        return outcome
    
    def strafe(self, key, userdata):
        strafe = self.strafes[key]
        self.offset_count += np.sign(strafe['angle'])
        rospy.loginfo("STRAFING %s to offset_count: %s" % (key, self.offset_count))
        userdata.active_strafe_key = key
        move_error = self.mover.execute_strafe(strafe['angle'], strafe['distance'])
        userdata.active_strafe_key = None
        rospy.loginfo("STRAFING %s returned blocked:%s, with distance error: %s" %(
                        key, strafe['blocked'], move_error))
    
  
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
    

    
    
    