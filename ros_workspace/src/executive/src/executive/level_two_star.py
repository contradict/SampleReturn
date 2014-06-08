#!/usr/bin/env python
import math
import collections
import threading
import random
from copy import deepcopy
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

from executive.executive_states import SelectMotionMode
from executive.executive_states import AnnounceState
from executive.executive_states import ExecuteSimpleMove
from executive.executive_states import DriveToPoint

import samplereturn.util as util
import samplereturn.bresenham as bresenham

class LevelTwoStar(object):
    
    def __init__(self):
        
        rospy.on_shutdown(self.shutdown_cb)
        self.node_params = util.get_node_params()
        self.tf_listener = tf.TransformListener()

        self.world_fixed_frame = rospy.get_param("world_fixed_frame", "map")
        self.odometry_frame = rospy.get_param("odometry_frame", "odom")
        
        #make a Point msg out of beacon_approach_point
        header = std_msg.Header(0, rospy.Time(0), self.world_fixed_frame)
        point = geometry_msg.Point(self.node_params.beacon_approach_point['x'],
                                   self.node_params.beacon_approach_point['y'], 0)
        self.beacon_approach_point = geometry_msg.PointStamped(header, point)
        #also need platform point
        platform_point = geometry_msg.Point( -0.4, 0, 0)
        self.platform_point = geometry_msg.PointStamped(header, platform_point)
        
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

        #get the star shape
        self.spokes = self.get_hollow_star(self.node_params.spoke_count,
                                           self.node_params.first_spoke_offset,
                                           self.node_params.star_hub_radius)
                       
        self.state_machine = smach.StateMachine(
                outcomes=['complete', 'preempted', 'aborted'],
                input_keys = ['action_goal'],
                output_keys = ['action_result'])
    
        self.state_machine.userdata.spokes = self.spokes
        self.state_machine.userdata.star_hub_radius = self.node_params.star_hub_radius
        self.state_machine.userdata.strafes = self.strafes
        self.state_machine.userdata.active_strafe_key = None
        #sets the default velocity used by ExecuteSimpleMove, if none, use simple_motion default
        self.state_machine.userdata.velocity = None

        #these are important values!  master frame id and return timing
        self.state_machine.userdata.world_fixed_frame = self.world_fixed_frame
        self.state_machine.userdata.odometry_frame = self.odometry_frame
        self.state_machine.userdata.start_time = rospy.Time.now()
        self.state_machine.userdata.return_time = rospy.Time.now() + \
                                                  rospy.Duration(self.node_params.return_time_minutes*60)
        self.state_machine.userdata.pre_cached_id = samplereturn_msg.NamedPoint.PRE_CACHED
        
        #beacon approach
        self.state_machine.userdata.beacon_approach_point = self.beacon_approach_point
        self.state_machine.userdata.platform_point = self.platform_point
        self.state_machine.userdata.offset_count = 0
        self.state_machine.userdata.offset_limit = 4
        self.state_machine.userdata.target_tolerance = 0.2 #both meters and radians for now!
        
        #search line parameters
        self.state_machine.userdata.spin_velocity = self.node_params.spin_velocity

        #search line variables
        self.state_machine.userdata.next_line_pose = None
        self.state_machine.userdata.last_line_pose = None
        self.state_machine.userdata.line_yaw = None #IN RADIANS!
        self.state_machine.userdata.outbound = False
        self.state_machine.userdata.distance_to_origin = 100
        self.state_machine.userdata.stop_on_sample = True
        self.state_machine.userdata.stop_on_beacon = False

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
                                                 output_keys=['action_result'],
                                                 outcomes=['next']),
                                   transitions = {'next':'ANNOUNCE_LEVEL_TWO'})
            
            smach.StateMachine.add('ANNOUNCE_LEVEL_TWO',
                                   AnnounceState(self.announcer,
                                                 'Enter ing level two mode'),
                                   transitions = {'next':'SELECT_PLANNER'})
            
            smach.StateMachine.add('SELECT_PLANNER',
                                    SelectMotionMode(self.CAN_interface,
                                                     MODE_PLANNER),
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
                                                  'return_home':'ANNOUNCE_RETURN_HOME',
                                                  'preempted':'LEVEL_TWO_PREEMPTED',
                                                  'aborted':'LEVEL_TWO_ABORTED'})
            
            smach.StateMachine.add('ROTATION_MANAGER',
                                   RotationManager(self.tf_listener, self.simple_mover),
                                   transitions = {'next':'ROTATE',
                                                  'preempted':'LEVEL_TWO_PREEMPTED',
                                                  'aborted':'LEVEL_TWO_ABORTED'})
            
            smach.StateMachine.add('ROTATE',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'LINE_MANAGER',
                                                  'timeout':'LINE_MANAGER',
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
                                                 'Return ing to home platform'),
                                   transitions = {'next':'BEACON_APPROACH'})
 
            smach.StateMachine.add('BEACON_APPROACH',
                                   BeaconApproach(self.tf_listener, self.announcer),
                                   transitions = {'mount':'MOUNT_MOVE',
                                                  'move':'BEACON_APPROACH_DRIVER',
                                                  'spin':'BEACON_APPROACH_SPIN',
                                                  'aborted':'LEVEL_TWO_ABORTED'})

            smach.StateMachine.add('BEACON_APPROACH_SPIN',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'BEACON_APPROACH',
                                                  'timeout':'BEACON_APPROACH',
                                                  'aborted':'LEVEL_TWO_ABORTED'})
            
            #return to start along the approach point
            #if the path is ever blocked just give up and return to the level_two search
            smach.StateMachine.add('BEACON_APPROACH_DRIVER',
                                   DriveToPoint(self.tf_listener, self.announcer),
                                   transitions = {'move':'BEACON_APPROACH_MOVE',
                                                  'detection_interrupt':'BEACON_APPROACH',
                                                  'blocked':'BEACON_APPROACH',
                                                  'complete':'BEACON_APPROACH'},
                                   remapping = {'detection_object':'beacon_point',
                                                'stop_on_detection':'stop_on_beacon'})

            smach.StateMachine.add('BEACON_APPROACH_MOVE',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'BEACON_APPROACH_DRIVER',
                                                  'timeout':'BEACON_APPROACH_DRIVER',
                                                  'aborted':'LEVEL_TWO_ABORTED'})
 
            smach.StateMachine.add('MOUNT_MOVE',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'DESELECT_PLANNER',
                                                  'timeout':'LEVEL_TWO_ABORTED',
                                                  'aborted':'LEVEL_TWO_ABORTED'})

            MODE_ENABLE = platform_srv.SelectMotionModeRequest.MODE_ENABLE
            smach.StateMachine.add('DESELECT_PLANNER',
                                    SelectMotionMode(self.CAN_interface,
                                                     MODE_PLANNER),
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
        
        if active_strafe_key is not None:
            if self.strafes[active_strafe_key]['blocked']:
                return True
            if not self.state_machine.userdata.outbound:
                current_pose = util.get_current_robot_pose(self.tf_listener,
                                                           self.world_fixed_frame)
                distance_to_origin = np.sqrt(current_pose.pose.position.x**2 +
                                             current_pose.pose.position.y**2)                
                if (distance_to_origin <= self.node_params.star_hub_radius):
                    self.state_machine.userdata.within_hub_radius = True
                    rospy.loginfo("LEVEL_TWO STOP simple_move on distance to origin = %.3f" %(distance_to_origin))
                    return True                
                                    
        if self.state_machine.userdata.stop_on_sample and \
                self.state_machine.userdata.detected_sample is not None:
            rospy.loginfo("LEVEL_TWO STOP simple_move on sample detection")
            return True
        
        if self.state_machine.userdata.stop_on_beacon and \
                self.state_machine.userdata.beacon_point is not None:
            rospy.loginfo("LEVEL_TWO STOP simple_move on beacon detection")
            return True
        
        return False

    def handle_costmap_check(self, costmap_check):
        for strafe_key, blocked in zip(costmap_check.strafe_keys,
                                       costmap_check.blocked):
            self.strafes[strafe_key]['blocked'] = blocked                        

    def pause_state_update(self, msg):
        self.state_machine.userdata.paused = msg.data
        
    def sample_update(self, sample):
        try:
            self.tf_listener.waitForTransform(self.odometry_frame,
                                              sample.header.frame_id,
                                              sample.header.stamp,
                                              rospy.Duration(1.0))
            point_in_frame = self.tf_listener.transformPoint(self.odometry_frame, sample)
            sample.point = point_in_frame.point
            self.state_machine.userdata.detected_sample = sample
        except tf.Exception:
            rospy.logwarn("LEVEL_TWO failed to transform search detection point %s->%s",
                          sample.header.frame_id, self.odometry_frame)
            
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
            
    def get_hollow_star(self, spoke_count, offset, hub_radius):

        offset = np.radians(offset)
        yaws = list(np.linspace(0 + offset, 2*np.pi + offset, spoke_count, endpoint=False))
        spokes = []
        
        for yaw in yaws:
            yaw = util.unwind(yaw)
            starting_point = geometry_msg.Point(hub_radius*math.cos(yaw),
                                                hub_radius*math.sin(yaw),
                                                0)
            spokes.append({'yaw':yaw,'starting_point':starting_point})
        
        #last star point is the beacon approach point    
        spokes.append({'yaw':None, 'starting_point':geometry_msg.Point(hub_radius, 0, 0)})
        
        return spokes
 
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

        return 'next'

class StarManager(smach.State):
    def __init__(self, tf_listener, announcer):
        smach.State.__init__(self,
                             input_keys = ['line_yaw',
                                           'outbound',
                                           'spokes',
                                           'world_fixed_frame'],
                             output_keys = ['line_yaw',
                                            'outbound',
                                            'spokes',
                                            'within_hub_radius'],
                             outcomes=['start_line',
                                       'rotate',
                                       'return_home',
                                       'preempted', 'aborted'])
        
        self.tf_listener = tf_listener
        self.announcer = announcer  

    def execute(self, userdata):
        #are we returning to the center?
        if not userdata.outbound:
            userdata.outbound = True
            if len(userdata.spokes) == 1:
                rospy.loginfo("STAR MANAGER finished inbound move, last spoke")
                return 'return_home'
            userdata.line_yaw = userdata.spokes.pop(0)['yaw']
            self.announcer.say("Start ing on spoke, Yaw %s" % (int(math.degrees(userdata.line_yaw))))
            return 'rotate'        
        
        if userdata.outbound:
            self.announcer.say("Return ing on spoke, Yaw " + str(int(math.degrees(userdata.line_yaw))))
            rospy.loginfo
            current_pose = util.get_current_robot_pose(self.tf_listener,
                                                       userdata.world_fixed_frame)
            userdata.line_yaw = util.pointing_yaw(current_pose.pose.position,
                                                  userdata.spokes[0]['starting_point'])
            rospy.loginfo("STAR_MANAGER returning to hub point: %s" %(userdata.spokes[0]['starting_point']))
            userdata.within_hub_radius = False
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
                                          'simple_move',
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
        #create the simple move command
        userdata.simple_move = {'type':'spin',
                                 'angle':rotate_yaw,
                                 'velocity':userdata.spin_velocity}
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
                                           'within_hub_radius'],
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

            #if userdata.paused:
            #    pause_start_time = rospy.Time.now()
            #    while not rospy.is_shutdown():
            #        if userdata.paused:
            #            rospy.sleep(0.2)
            #        else:
            #            break
            
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
 
            #are we within the star hub radius?
            if not userdata.outbound and userdata.within_hub_radius:
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
    
  
class BeaconApproach(smach.State):
 
    def __init__(self, tf_listener, announcer):

        smach.State.__init__(self,
                             outcomes=['move',
                                       'spin',
                                       'mount',
                                       'aborted'],
                             input_keys=['beacon_approach_point',
                                         'platform_point',
                                         'beacon_point',
                                         'stop_on_beacon',
                                         'world_fixed_frame'],
                             output_keys=['target_point',
                                          'target_yaw',
                                          'target_tolerance',
                                          'offset_count',
                                          'simple_move',
                                          'stop_on_sample',
                                          'stop_on_beacon',
                                          'beacon_point'])
        
        self.tf_listener = tf_listener
        self.announcer = announcer
        self.tried_spin = False

    def execute(self, userdata):
        
        #reset offset counter for driving moves
        userdata.offset_count = 0
        #ignore samples now
        userdata.stop_on_sample = False
        #by default, driving moves won't have a target_yaw
        userdata.target_yaw = None

        #if we have been ignoring beacon detections prior to this,
        #we should clear them here, and wait for a fresh detection
        if not userdata.stop_on_beacon:
            userdata.beacon_point = None
            rospy.sleep(4.0)
        
        if userdata.beacon_point is None: #beacon not in view
            #first hope, we can spin slowly and see the beacon
            if not self.tried_spin:
                self.announcer.say("Beacon not in view. Rotate ing")
                userdata.simple_move = {'type':'spin',
                                        'angle':math.pi*2,
                                        'velocity':0.25}
                userdata.stop_on_beacon = True
                self.tried_spin = True
                return 'spin'
            #already tried a spin, drive towards beacon_approach_point, stopping on detection
            else:
                #gotta add some heroic shit here to search around for beacon       
                self.announcer.say("Beacon not in view. Search ing")
                userdata.target_point = deepcopy(userdata.beacon_approach_point)
                userdata.target_yaw = math.pi
                userdata.stop_on_beacon = True
                self.tried_spin = False
                return 'move' 
        else: #beacon is in view
            #need to add some stuff here to get to other side of beacon if viewing back
            current_pose = util.get_current_robot_pose(self.tf_listener,
                                                       userdata.world_fixed_frame)
            current_yaw = util.get_current_robot_yaw(self.tf_listener,
                                                       userdata.world_fixed_frame)
            yaw_to_platform = util.pointing_yaw(current_pose.pose.position,
                                                userdata.platform_point.point)
            yaw_error = yaw_to_platform - current_yaw
            distance_to_approach_point = util.point_distance_2d(current_pose.pose.position,
                                                                userdata.beacon_approach_point.point)
            userdata.stop_on_beacon = False
            self.tried_spin = False            
            #on the back side
            if current_pose.pose.position.x < 0:
                front_point = deepcopy(userdata.beacon_approach_point)
                #try not to have to drive through the platform
                front_point.point.y = 5.0 * np.sign(current_pose.pose.position.y)
                userdata.target_point = front_point
                userdata.target_yaw = math.pi
                self.announcer.say("Back of beacon in view. Move ing to front")
                return 'move'   
            elif distance_to_approach_point > 1.0:
                #on correct side of beacon, but far from approach point
                userdata.target_point = deepcopy(userdata.beacon_approach_point)
                userdata.target_yaw = math.pi
                self.announcer.say("Beacon in view. Move ing to approach point")                
                return 'move'   
            elif np.abs(yaw_error) > .05:
                #this means beacon is in view and we are within 1 meter of approach point
                #but not pointed so well at beacon
                self.announcer.say("At approach point. Aligning to beacon")
                userdata.simple_move = {'type':'spin',
                                        'angle':yaw_error,
                                        'velocity':0.5}
                return 'spin'
            else:    
                self.announcer.say("Mount ing platform")
                #make this a parameter probably
                distance_to_platform = util.point_distance_2d(current_pose.pose.position,
                                                              userdata.platform_point.point)
                userdata.simple_move = {'type':'strafe',
                                        'angle':yaw_error,
                                        'distance':distance_to_platform,
                                        'velocity':0.5}                
                return 'mount'
        
        return 'aborted'
 
    
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
    

    
    
    