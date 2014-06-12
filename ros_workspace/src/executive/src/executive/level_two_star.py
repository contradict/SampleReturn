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
from executive.executive_states import RotateToClear

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
        platform_point = geometry_msg.Point( 0, 0, 0)
        self.platform_point = geometry_msg.PointStamped(header, platform_point)
        
        #interfaces
        self.announcer = util.AnnouncerInterface("audio_search")
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

        #these are important values! master frame id and return timing
        self.state_machine.userdata.world_fixed_frame = self.world_fixed_frame
        self.state_machine.userdata.odometry_frame = self.odometry_frame
        self.state_machine.userdata.start_time = rospy.Time.now()
        self.state_machine.userdata.return_time = rospy.Time.now() + \
                                                  rospy.Duration(self.node_params.return_time_minutes*60)
        self.state_machine.userdata.pre_cached_id = samplereturn_msg.NamedPoint.PRE_CACHED
        
        #beacon approach
        self.state_machine.userdata.beacon_approach_point = self.beacon_approach_point
        self.state_machine.userdata.beacon_mount_step = self.node_params.beacon_mount_step
        self.state_machine.userdata.platform_point = self.platform_point
        self.state_machine.userdata.offset_count = 0
        self.state_machine.userdata.offset_limit = 0
        self.state_machine.userdata.target_tolerance = 0.2 #both meters and radians for now!
        
        #search line parameters
        self.state_machine.userdata.min_spin_radius = self.node_params.min_spin_radius
        self.state_machine.userdata.last_spin_radius = 0
        self.state_machine.userdata.spin_step = self.node_params.spin_step
        self.state_machine.userdata.spin_velocity = self.node_params.spin_velocity
        self.state_machine.userdata.line_yaw = None #IN RADIANS!
        self.state_machine.userdata.outbound = False
        self.state_machine.userdata.distance_to_origin = 100
        
        #stop function flags
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
                                                 output_keys=['action_result',
                                                              'dismount_move'],
                                                 outcomes=['next']),
                                   transitions = {'next':'ANNOUNCE_LEVEL_TWO'})
            
            smach.StateMachine.add('ANNOUNCE_LEVEL_TWO',
                                   AnnounceState(self.announcer,
                                                 'Enter ing level two mode'),
                                   transitions = {'next':'SELECT_PLANNER'})
            
            smach.StateMachine.add('SELECT_PLANNER',
                                    SelectMotionMode(self.CAN_interface,
                                                     MODE_PLANNER),
                                    transitions = {'next':'DISMOUNT_MOVE',
                                                  'failed':'LEVEL_TWO_ABORTED'})
            
            smach.StateMachine.add('DISMOUNT_MOVE',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'STAR_MANAGER',
                                                  'timeout':'STAR_MANAGER',
                                                  'aborted':'LEVEL_TWO_ABORTED'},
                                   remapping = {'simple_move':'dismount_move'})
            
            smach.StateMachine.add('STAR_MANAGER',
                                   StarManager(self.tf_listener,
                                               self.announcer),
                                   transitions = {'rotate':'ROTATION_MANAGER',
                                                  'return_home':'ANNOUNCE_RETURN_HOME'})

            smach.StateMachine.add('LINE_MANAGER',
                                   SearchLineManager(self.tf_listener,
                                                     self.simple_mover,
                                                     self.announcer),
                                   transitions = {'move':'LINE_MOVE',
                                                  'spin':'ROTATE',
                                                  'recalibrate':'ROTATION_MANAGER',
                                                  'sample_detected':'PURSUE_SAMPLE',
                                                  'line_blocked':'STAR_MANAGER',
                                                  'next_spoke':'STAR_MANAGER',
                                                  'return_home':'ANNOUNCE_RETURN_HOME',
                                                  'preempted':'LEVEL_TWO_PREEMPTED',
                                                  'aborted':'LEVEL_TWO_ABORTED'})

            smach.StateMachine.add('LINE_MOVE',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'LINE_MANAGER',
                                                  'timeout':'LINE_MANAGER',
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
                                                  'aborted':'LEVEL_TWO_ABORTED'},
                                   remapping = {'velocity':'spin_velocity'})
            
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
                                   transitions = {'next':'BEACON_SEARCH'})
 
            smach.StateMachine.add('BEACON_SEARCH',
                                   BeaconSearch(self.tf_listener, self.announcer),
                                   transitions = {'mount':'MOUNT_MANAGER',
                                                  'move':'BEACON_SEARCH_DRIVER',
                                                  'spin':'BEACON_SEARCH_SPIN',
                                                  'aborted':'LEVEL_TWO_ABORTED'})
 
            smach.StateMachine.add('BEACON_SEARCH_SPIN',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'BEACON_SEARCH',
                                                  'timeout':'BEACON_SEARCH',
                                                  'aborted':'LEVEL_TWO_ABORTED'})
            
            #return to start along the approach point
            #if the path is ever blocked just give up and return to the level_two search
            smach.StateMachine.add('BEACON_SEARCH_DRIVER',
                                   DriveToPoint(self.tf_listener, self.announcer),
                                   transitions = {'move':'BEACON_SPIN_MOVE',
                                                  'detection_interrupt':'BEACON_SEARCH',
                                                  'blocked':'BEACON_CLEAR_MOVE',
                                                  'complete':'BEACON_SEARCH'},
                                   remapping = {'detection_object':'beacon_point',
                                                'stop_on_detection':'stop_on_beacon'})

            smach.StateMachine.add('BEACON_SPIN_MOVE',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'BEACON_SEARCH_DRIVER',
                                                  'timeout':'BEACON_SEARCH_DRIVER',
                                                  'aborted':'LEVEL_TWO_ABORTED'})

            smach.StateMachine.add('BEACON_CLEAR_MOVE',
                                   RotateToClear(self.simple_mover, self.tf_listener, self.strafes),
                                   transitions = {'complete':'BEACON_SEARCH',
                                                  'blocked':'BEACON_SEARCH',
                                                  'aborted':'LEVEL_TWO_ABORTED'})
 
            smach.StateMachine.add('MOUNT_MANAGER',
                                   MountManager(self.tf_listener, self.announcer),
                                   transitions = {'move':'MOUNT_MOVE',
                                                  'final':'MOUNT_FINAL',
                                                  'aborted':'LEVEL_TWO_ABORTED'})
 
            smach.StateMachine.add('MOUNT_MOVE',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'MOUNT_MANAGER',
                                                  'timeout':'MOUNT_MANAGER',
                                                  'aborted':'LEVEL_TWO_ABORTED'})
 
            smach.StateMachine.add('MOUNT_FINAL',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'DESELECT_PLANNER',
                                                  'timeout':'MOUNT_MANAGER',
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
                        geometry_msg.PoseWithCovarianceStamped,
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
        
        #the pause switch stops the robot, but we must tell the simple_mover it is stopped
        if self.state_machine.userdata.paused:
            rospy.loginfo("LEVEL TWO STOP: on pause")
            return True
        
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
                                                 beacon_pose.pose.pose.position)
        self.state_machine.userdata.beacon_point = beacon_point

    def get_hollow_star(self, spoke_count, offset, hub_radius):

        offset = np.radians(offset)
        yaws = list(np.linspace(0 + offset, -2*np.pi + offset, spoke_count, endpoint=False))
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
        
        userdata.dismount_move = {'type':'strafe',
                                   'angle':0,
                                   'distance':5,
                                   'velocity':0.2}

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
                                            'offset_count',
                                            'offset_limit',
                                            'last_spin_radius',
                                            'within_hub_radius'],
                             outcomes=['rotate',
                                       'return_home',
                                       'preempted', 'aborted'])
        
        self.tf_listener = tf_listener
        self.announcer = announcer  

    def execute(self, userdata):
        
        #returning to the center
        if not userdata.outbound:
            #just finished inbound spoke, turn around and head outwards again
            if len(userdata.spokes) == 1:
                rospy.loginfo("STAR MANAGER finished inbound move, last spoke")
                return 'return_home'
            userdata.last_spin_radius = util.get_robot_distance_to_origin(self.tf_listener,
                                                                          userdata.world_fixed_frame)
            userdata.line_yaw = userdata.spokes.pop(0)['yaw']
            userdata.offset_count = 0
            userdata.offset_limit = 1
            userdata.outbound = True
            rospy.loginfo("STAR_MANAGER starting spoke: %.2f" %(userdata.line_yaw))
            self.announcer.say("Start ing on spoke, Yaw %s" % (int(math.degrees(userdata.line_yaw))))
            return 'rotate'        
        #outbound
        else:
            #just finished outbound spoke, head towards next star point and set offset limit higher.
            #If inbound, rotation manager calculates the line yaw each time it is asked to make a
            #rotation. userdata.line_yaw remains the yaw FROM the origin
            userdata.within_hub_radius = False
            userdata.offset_count = 0
            userdata.offset_limit = 2
            userdata.outbound = False
            rospy.loginfo("STAR_MANAGER returning to hub point: %s" %(userdata.spokes[0]['starting_point']))
            self.announcer.say("Return ing on spoke, Yaw " + str(int(math.degrees(userdata.line_yaw))))
            return 'rotate'


class RotationManager(smach.State):

    def __init__(self, tf_listener, mover):
        smach.State.__init__(self,
                             outcomes=['next', 'preempted', 'aborted'],
                             input_keys=['line_yaw',
                                         'spokes',
                                         'spin_velocity',
                                         'outbound',
                                         'world_fixed_frame'],
                             output_keys=['line_yaw',
                                          'simple_move',
                                          'outbound',
                                          'rotate_pose',
                                          'beacon_point',
                                          'last_align_time']),  
  
        self.tf_listener = tf_listener
        self.mover = mover
    
    def execute(self, userdata):
        #if heading out, always move parallel to line yaw
        if userdata.outbound:
            map_yaw = deepcopy(userdata.line_yaw)
        #if inbound, always point to next starting point
        else:
            current_pose = util.get_current_robot_pose(self.tf_listener,
                                                       userdata.world_fixed_frame)
            map_yaw = util.pointing_yaw(current_pose.pose.position,
                                                  userdata.spokes[0]['starting_point'])
        
        actual_yaw = util.get_current_robot_yaw(self.tf_listener,
                userdata.world_fixed_frame)
        rotate_yaw = util.unwind(map_yaw - actual_yaw)
        rospy.loginfo("ROTATION MANAGER map_yaw: %.1f, robot_yaw: %.1f, rotate_yaw: %.1f" %(
                      np.degrees(map_yaw),
                      np.degrees(actual_yaw),
                      np.degrees(rotate_yaw)))
        #create the simple move command
        userdata.simple_move = {'type':'spin',
                                 'angle':rotate_yaw,
                                 'velocity':userdata.spin_velocity}
        #clear the beacon points before returning, make sure we respond to new beacon poses
        userdata.last_align_time = rospy.get_time()
        userdata.beacon_point = None
        return 'next'

#drive to detected sample location        
class SearchLineManager(smach.State):
    def __init__(self, tf_listener, mover, announcer):
        smach.State.__init__(self,
                             input_keys = ['strafes',
                                           'active_strafe_key',
                                           'line_yaw',
                                           'outbound',
                                           'offset_count',
                                           'offset_limit',
                                           'return_time',
                                           'detected_sample',
                                           'world_fixed_frame',
                                           'odometry_frame',
                                           'last_spin_radius',
                                           'min_spin_radius',
                                           'spin_step',
                                           'within_hub_radius',
                                           'beacon_point',
                                           'last_align_time'],
                             output_keys = ['simple_move',
                                            'offset_count',
                                            'active_strafe_key',
                                            'last_spin_radius',
                                            'beacon_point',
                                            'last_align_time'],
                             outcomes=['move',
                                       'spin',
                                       'recalibrate',
                                       'sample_detected',
                                       'line_blocked',
                                       'next_spoke',
                                       'return_home',
                                       'preempted', 'aborted'])
        
        self.tf_listener = tf_listener
        self.mover = mover
        self.announcer = announcer

    def execute(self, userdata):
    
        userdata.active_strafe_key = None
        
        actual_yaw = util.get_current_robot_yaw(self.tf_listener,
                userdata.world_fixed_frame)
        current_pose = util.get_current_robot_pose(self.tf_listener,
                userdata.world_fixed_frame)
        current_radius = util.point_distance_2d(current_pose.pose.position,
                                                geometry_msg.Point(0,0,0))
        
        rospy.loginfo("SEARCH LINE MANAGER, outbound: %s,  line_yaw: %.1f,  actual_yaw: %.1f,  position: (%.2f,%.2f) " % (
                       userdata.outbound,
                       np.degrees(userdata.line_yaw),
                       np.degrees(actual_yaw),
                       current_pose.pose.position.x,
                       current_pose.pose.position.y))
        
        #useful condition lambdas and other flags for dumb planner behaviour
        under_offset_limit = lambda: np.abs(userdata.offset_count) < userdata.offset_limit
        first_angle_choice = 'left' if userdata.outbound else 'right'
        second_angle_choice = 'right' if userdata.outbound else 'left'
        
        if rospy.Time.now() > userdata.return_time:
            self.announcer.say("Search time expired")
            return 'return_home'
        
        #check preempt after deciding whether or not to calculate next line pose
        if self.preempt_requested():
            rospy.loginfo("LINE MANAGER: preempt requested")
            self.service_preempt()
            return 'preempted'
        
        if userdata.detected_sample is not None:
            return "sample_detected"

        #are we within the star hub radius?
        if not userdata.outbound and userdata.within_hub_radius:
            return 'next_spoke'     

        #time for a spin?
        if userdata.outbound \
           and (current_radius > userdata.min_spin_radius) \
           and (current_radius - userdata.last_spin_radius) > userdata.spin_step:
            self.announcer.say("Scan ing for samples")
            userdata.last_spin_radius = current_radius
            userdata.simple_move = {'type':'spin',
                                    'angle':2*math.pi}
            return 'spin'
        
        if (not userdata.outbound) and (userdata.beacon_point is not None) \
            and not False in userdata.strafes and ((rospy.get_time() - userdata.last_align_time) > 10):
            #if we are seeing the beacon, and are not making some kind of move because we're blocked,
            #adjust our yaw to map coords.  Hopefully these are small rotations....
            self.announcer.say("Align ing to map")
            userdata.last_align_time = rospy.get_time()
            userdata.beacon_point = None
            return 'recalibrate'
        
        
        #did we get here from a rotation, if so, pause to wait for costmaps
        if userdata.active_strafe_key is None:
            rospy.sleep(3.0)        
        
        #BEGINNING OF THE MIGHTY LINE MANAGER CASE       
        #first check if we are offset from line either direction, and if so, is the path
        #back the line clear.  If is is, strafe towards the line
        if (userdata.offset_count > 0) and not userdata.strafes['right']['blocked']:
            #left of line
            self.announcer.say('Right clear, strafe ing to line')
            return self.strafe('right', userdata)
        elif (userdata.offset_count < 0) and not userdata.strafes['left']['blocked']:
            #right of line
            self.announcer.say('Left clear, strafe ing to line')
            return self.strafe('left', userdata)
        
        #if not offset or returning to line is blocked, going straight is next priority
        elif not userdata.strafes['center']['blocked']:
            self.announcer.say("Line clear, continue ing")    
            return self.strafe('center', userdata)
           
        #if returning to the line not possible or necessary,
        #and going straight isn't possible: offset from line             
        elif not userdata.strafes[first_angle_choice]['blocked'] and under_offset_limit():
            self.announcer.say("Obstacle in line, strafe ing %s" % (first_angle_choice))
            return self.strafe(first_angle_choice, userdata)
            
        elif not userdata.strafes[second_angle_choice]['blocked'] and under_offset_limit():
            self.announcer.say("Obstacle in line, strafe ing %s" % (second_angle_choice))
            return self.strafe(second_angle_choice, userdata)
        
        #getting here means lethal cells in all three yaw check lines,
        #or that the only unblocked line takes us outside our allowable offset
        else:             
            self.announcer.say("Line is blocked, rotate ing")
            return 'line_blocked'
        
        return 'aborted'
    
    def strafe(self, key, userdata):
        strafe = userdata.strafes[key]
        userdata.offset_count += np.sign(strafe['angle'])
        rospy.loginfo("STRAFING %s to offset_count: %s" % (key, userdata.offset_count))
        userdata.simple_move = {'type':'strafe',
                                'angle':strafe['angle'],
                                'distance': strafe['distance']}
        userdata.active_strafe_key = key
        return 'move'
  
class BeaconSearch(smach.State):
 
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
                                          'offset_limit',
                                          'simple_move',
                                          'stop_on_sample',
                                          'stop_on_beacon',
                                          'beacon_point',
                                          'clear_spin',
                                          'clear_move',
                                          'outbound'])
        
        self.tf_listener = tf_listener
        self.announcer = announcer
        self.tried_spin = False
        self.clear_move = {'type':'strafe',
                           'angle':0,
                           'distance': 20.0}
        self.clear_spin = {'type':'spin',
                           'angle':math.pi*2,
                           'velocity': 0.2}

    def execute(self, userdata):
        
        #this is lame as hell, but to disable simple_mover stop function returning true
        #from here on out, easiest thing to do is set outbound back to true
        userdata.outbound = True 
        
        #reset offset counter for driving moves
        userdata.offset_count = 0
        userdata.offset_limit = 3
        userdata.clear_spin = self.clear_spin
        userdata.clear_move = self.clear_move
        #ignore samples now
        userdata.stop_on_sample = False
        #by default, driving moves won't have a target_yaw
        userdata.target_yaw = None
        
        current_pose = util.get_current_robot_pose(self.tf_listener,
                                                   userdata.world_fixed_frame)        
        #this is possibly a highly inaccurate number.   If we get to the approach point,
        #and don't see the beacon, the map is probably messed up
        distance_to_approach_point = util.point_distance_2d(current_pose.pose.position,
                                                            userdata.beacon_approach_point.point)

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
                                        'velocity':0.15}
                userdata.stop_on_beacon = True
                self.tried_spin = True
                return 'spin'
            #already tried a spin, drive towards beacon_approach_point, stopping on detection
            elif distance_to_approach_point > 5.0:
                #we think we're far from approach_point, so try to go there
                self.announcer.say("Beacon not in view. Search ing")
                userdata.target_point = deepcopy(userdata.beacon_approach_point)
                userdata.target_yaw = math.pi
                userdata.stop_on_beacon = True
                self.tried_spin = False
                return 'move'
            else:
                #gotta add some heroic shit here
                #we think we're near the beacon, but we don't see it
                #right now, that is just to move 30 m on other side of the beacon that we don't know about
                self.announcer.say("Close to approach point in map.  Beacon not in view.  Search ing")
                search_point = deepcopy(userdata.beacon_approach_point)                
                #invert the approach_point, and try again
                search_point.point.x *= -1
                userdata.target_point = search_point
                userdata.stop_on_beacon = True
                self.tried_spin = False
                return 'move'               
                
        else: #beacon is in view
            #need to add some stuff here to get to other side of beacon if viewing back
            current_yaw = util.get_current_robot_yaw(self.tf_listener,
                                                       userdata.world_fixed_frame)
            yaw_to_platform = util.pointing_yaw(current_pose.pose.position,
                                                userdata.platform_point.point)
            yaw_error = util.unwind(yaw_to_platform - current_yaw)
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
                self.announcer.say("At approach point. Rotate ing to beacon")
                userdata.simple_move = {'type':'spin',
                                        'angle':yaw_error,
                                        'velocity':0.3}
                return 'spin'
            else:    
                self.announcer.say("Initiate ing platform mount")
                userdata.stop_on_beacon = False
                return 'mount'
        
        return 'aborted'
    
class MountManager(smach.State):
    
    def __init__(self, tf_listener, announcer):

        smach.State.__init__(self,
                             outcomes=['move',
                                       'final',
                                       'aborted'],
                             input_keys=['platform_point',
                                         'beacon_point',
                                         'beacon_mount_step'],
                             output_keys=['simple_move',
                                          'beacon_point',
                                          'active_strafe_key'])

        self.tf_listener = tf_listener
        self.announcer = announcer

    def execute(self, userdata):
        #disable obstacle checking!
        rospy.sleep(2.0) #wait a sec for beacon pose to catch up
        userdata.active_strafe_key = None
        yaw, distance = util.get_robot_strafe(self.tf_listener,
                                             userdata.platform_point)
        if (userdata.beacon_point is None) or (distance < userdata.beacon_mount_step):
            #we don't see the beacon anymore, or we see it and are damn close (doubtful!)
            userdata.simple_move = {'type':'strafe',
                                    'angle':yaw,
                                    'distance':distance,
                                    'velocity':0.5}
            self.announcer.say("Execute ing final mount move")
            return 'final'
        else:
            userdata.simple_move = {'type':'strafe',
                                    'angle':yaw,
                                    'distance':userdata.beacon_mount_step,
                                    'velocity':0.5}
            self.announcer.say("Aligning to beacon")
            userdata.beacon_point = None #clear beacon returns to see if we are still seeing it
            return 'move'
    
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
    

    
    
    
