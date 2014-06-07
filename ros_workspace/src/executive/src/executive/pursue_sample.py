#!/usr/bin/env python
import math
import collections
import threading
import numpy as np

import smach
import smach_ros
import rospy
import actionlib
import tf

import std_msgs.msg as std_msg
import actionlib_msgs.msg as action_msg
import visual_servo_msgs.msg as visual_servo_msg
import samplereturn_msgs.msg as samplereturn_msg
import platform_motion_msgs.msg as platform_msg
import platform_motion_msgs.srv as platform_srv
import manipulator_msgs.msg as manipulator_msg
import move_base_msgs.msg as move_base_msg
import geometry_msgs.msg as geometry_msg
import samplereturn_msgs.srv as samplereturn_srv
import visualization_msgs.msg as vis_msg

import samplereturn.util as util
import samplereturn.simple_motion as simple_motion
from samplereturn.simple_motion import TimeoutException

from executive.executive_states import DriveToPoseState
from executive.executive_states import WaitForFlagState
from executive.executive_states import AnnounceState
from executive.executive_states import GetPursueDetectedPointState
from executive.executive_states import SelectMotionMode
from executive.executive_states import GetSimpleMoveState
from executive.executive_states import ServoController
from executive.executive_states import ExecuteSimpleMove

class PursueSample(object):
    
    def __init__(self):
        
        rospy.on_shutdown(self.shutdown_cb)
        self.node_params = util.get_node_params()
        self.tf_listener = tf.TransformListener()

        self.world_fixed_frame = rospy.get_param("world_fixed_frame", "map")
        self.odometry_frame = rospy.get_param("odometry_frame", "odom")
        
        #interfaces
        self.announcer = util.AnnouncerInterface("audio_navigate")
        self.result_pub = rospy.Publisher('pursuit_result', samplereturn_msg.PursuitResult)
        self.light_pub = rospy.Publisher('search_lights', std_msg.Bool)
        self.CAN_interface = util.CANInterface()
        
        #get a simple_mover, it's parameters are inside a rosparam tag for this node
        self.simple_mover = simple_motion.SimpleMover('~sample_search_params/',
                                                      self.tf_listener,
                                                      self.check_for_stop)
        
        #strafe definitions, offset is length along strafe line
        #the yaws have a static angle, which the direction from base_link the robot strafes
        self.strafes = rospy.get_param('strafes')
        self.strafes['center']['distance'] = self.node_params.pursuit_step
        
        #for this state machine, there is no preempt path.  It either finshes successfully and
        #reports success on the PursuitResult topic, or in case of any interupption or failure, it
        #exits through the abort path, reporting failure on the topic.
        self.state_machine = smach.StateMachine(
                outcomes=['complete', 'preempted', 'aborted'],
                input_keys = ['action_goal'],
                output_keys = ['action_result'])
        
        self.state_machine.userdata.odometry_frame = self.odometry_frame
        self.state_machine.userdata.world_fixed_frame = self.world_fixed_frame

        self.state_machine.userdata.strafes = self.strafes
        self.state_machine.userdata.active_strafe_key = None
        
        self.state_machine.userdata.paused = False        
        self.state_machine.userdata.detected_sample = None
        self.state_machine.userdata.target_sample = None
        self.state_machine.userdata.search_point = None
        self.state_machine.latched_sample = None
        
        self.state_machine.userdata.pursuit_velocity = self.node_params.pursuit_velocity
        self.state_machine.userdata.pursuit_step = self.node_params.pursuit_step
        self.state_machine.userdata.check_pursuit_distance = False
        self.state_machine.userdata.max_pursuit_error = self.node_params.max_pursuit_error       
        self.state_machine.userdata.min_pursuit_distance = self.node_params.min_pursuit_distance
        self.state_machine.userdata.max_sample_lost_time = self.node_params.max_sample_lost_time        

        #strafe search and servo settings, search_count limits the number of times we can enter the
        #sample_lost search pattern
        self.state_machine.userdata.search_velocity = self.node_params.search_velocity
        self.state_machine.userdata.square_search_size = self.node_params.square_search_size
        self.state_machine.userdata.settle_time = 5
        self.state_machine.userdata.manipulator_search_angle = math.pi/4
        self.state_machine.userdata.search_count = 0
        self.state_machine.userdata.search_try_limit = 2
        self.state_machine.userdata.manipulator_correction = self.node_params.manipulator_correction
        self.state_machine.userdata.servo_params = self.node_params.servo_params
        
        #use these as booleans in remaps
        self.state_machine.userdata.true = True
        self.state_machine.userdata.false = False

        #motion mode stuff
        planner_mode = self.node_params.planner_mode
        MODE_PLANNER = getattr(platform_srv.SelectMotionModeRequest, planner_mode)
        MODE_SERVO = platform_srv.SelectMotionModeRequest.MODE_SERVO            

        with self.state_machine:
            
            smach.StateMachine.add('START_SAMPLE_PURSUIT',
                                   StartSamplePursuit(),
                                   transitions = {'next':'APPROACH_SAMPLE'})

            smach.StateMachine.add('APPROACH_SAMPLE',
                                   ApproachPoint(self.tf_listener, self.announcer),
                                   transitions = {'complete':'ANNOUNCE_MANIPULATOR_APPROACH',
                                                  'move':'APPROACH_MOVE',
                                                  'blocked':'PUBLISH_FAILURE',
                                                  'point_lost':'PUBLISH_FAILURE',
                                                  'aborted':'PURSUE_SAMPLE_ABORTED'})
                                               
            smach.StateMachine.add('APPROACH_MOVE',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'APPROACH_SAMPLE',
                                                  'blocked':'APPROACH_SAMPLE',
                                                  'timeout':'APPROACH_SAMPLE',
                                                  'aborted':'APPROACH_SAMPLE'},
                                   remapping = {'velocity':'pursuit_velocity'})
           
            smach.StateMachine.add('ANNOUNCE_MANIPULATOR_APPROACH',
                                   AnnounceState(self.announcer,
                                                 'Calculate ing manipulator approach'),
                                   transitions = {'next':'ENABLE_MANIPULATOR_DETECTOR'})                

            smach.StateMachine.add('ENABLE_MANIPULATOR_DETECTOR',
                                    smach_ros.ServiceState('enable_manipulator_detector',
                                                            samplereturn_srv.Enable,
                                                            request = samplereturn_srv.EnableRequest(True)),
                                     transitions = {'succeeded':'GET_SAMPLE_STRAFE_MOVE',
                                                    'aborted':'PUBLISH_FAILURE'})
 
            #calculate the final strafe move to the sample
            smach.StateMachine.add('GET_SAMPLE_STRAFE_MOVE',
                                   GetManipulatorApproachMove(self.tf_listener),
                                   transitions = {'strafe':'MANIPULATOR_APPROACH',
                                                  'point_lost':'PUBLISH_FAILURE'})
            
            smach.StateMachine.add('MANIPULATOR_APPROACH',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'HANDLE_SEARCH',
                                                  'blocked':'HANDLE_SEARCH',
                                                  'timeout':'PURSUE_SAMPLE_ABORTED',
                                                  'aborted':'PUBLISH_FAILURE'},
                                   remapping = {'velocity':'search_velocity'})
            
            smach.StateMachine.add('HANDLE_SEARCH',
                                   HandleSearch(self.tf_listener, self.announcer),
                                   transitions = {'sample_search':'HANDLE_SEARCH_MOVES',
                                                  'sample_detected':'VISUAL_SERVO',
                                                  'aborted':'PUBLISH_FAILURE'})

            smach.StateMachine.add('HANDLE_SEARCH_MOVES',
                                   MoveToPoints(self.tf_listener),
                                   transitions = {'next_point':'SEARCH_MOVE',
                                                  'sample_detected':'VISUAL_SERVO',
                                                  'complete':'ANNOUNCE_SEARCH_FAILURE'},
                                   remapping = {'face_next_point':'false',
                                                'obstacle_check':'false'})
   
            smach.StateMachine.add('SEARCH_MOVE',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'HANDLE_SEARCH_MOVES',
                                                  'blocked':'HANDLE_SEARCH_MOVES',
                                                  'timeout':'HANDLE_SEARCH_MOVES',
                                                  'aborted':'PUBLISH_FAILURE'},
                                   remapping = {'velocity':'search_velocity'})

            smach.StateMachine.add('ANNOUNCE_SEARCH_FAILURE',
                                   AnnounceState(self.announcer,
                                                 "Search complete, no sample found"),
                                   transitions = {'next':'PUBLISH_FAILURE'})
          
            #calculate the strafe move to the sample
            smach.StateMachine.add('VISUAL_SERVO',
                                   ServoController(self.tf_listener, self.announcer),
                                   transitions = {'move':'SERVO_MOVE',
                                                  'complete':'GRAB_SAMPLE',
                                                  'point_lost':'HANDLE_SEARCH',
                                                  'aborted':'PUBLISH_FAILURE'})

            smach.StateMachine.add('SERVO_MOVE',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'VISUAL_SERVO',
                                                  'blocked':'VISUAL_SERVO',
                                                  'timeout':'VISUAL_SERVO',
                                                  'aborted':'PUBLISH_FAILURE'})

            @smach.cb_interface(input_keys=['latched_sample'])
            def grab_goal_cb(userdata, request):
                goal = manipulator_msg.ManipulatorGoal()
                goal.type = goal.GRAB
                goal.wrist_angle = userdata.latched_sample.grip_angle                    
                goal.target_bin = userdata.latched_sample.sample_id
                goal.grip_torque = 0.7
                return goal
    
            #if Steve pauses the robot during this action, it returns preempted,
            #return to visual servo after pause, in case robot moved
            smach.StateMachine.add('GRAB_SAMPLE',
                                   smach_ros.SimpleActionState('manipulator_action',
                                   manipulator_msg.ManipulatorAction,
                                   goal_cb = grab_goal_cb),
                                   transitions = {'succeeded':'CONFIRM_SAMPLE_ACQUIRED',
                                                  'preempted':'CHECK_PAUSE_STATE',
                                                  'aborted':'PUBLISH_FAILURE'})

            smach.StateMachine.add('CONFIRM_SAMPLE_ACQUIRED',
                                   ConfirmSampleAcquired(self.announcer, self.result_pub),
                                   transitions = {'sample_gone':'DISABLE_MANIPULATOR_DETECTOR',
                                                  'sample_present':'VISUAL_SERVO',
                                                  'aborted':'PUBLISH_FAILURE'})

            #if the grab action is preempted by shutdown (or other non-pause reason),
            #exit pursue sample.  If paused, wait for unpause
            smach.StateMachine.add('CHECK_PAUSE_STATE',
                                   WaitForFlagState('paused',
                                                    flag_trigger_value = True),
                                   transitions = {'next':'WAIT_FOR_UNPAUSE',
                                                  'timeout':'PUBLISH_FAILURE',
                                                  'preempted':'PUBLISH_FAILURE'})
            
            smach.StateMachine.add('WAIT_FOR_UNPAUSE',
                                   WaitForFlagState('paused',
                                                    flag_trigger_value = False,
                                                    timeout = 20,
                                                    announcer = self.announcer,
                                                    start_message ='Pursuit paused, waiting for un pause'),
                                   transitions = {'next':'VISUAL_SERVO',
                                                  'timeout':'WAIT_FOR_UNPAUSE',
                                                  'preempted':'PUBLISH_FAILURE'})


            smach.StateMachine.add('PUBLISH_FAILURE',
                                   PublishFailure(self.result_pub),
                                   transitions = {'next':'DISABLE_MANIPULATOR_DETECTOR'})

            #beginning of clean exit path
            smach.StateMachine.add('DISABLE_MANIPULATOR_DETECTOR',
                                    smach_ros.ServiceState('enable_manipulator_detector',
                                                            samplereturn_srv.Enable,
                                                            request = samplereturn_srv.EnableRequest(False)),
                                     transitions = {'succeeded':'ANNOUNCE_RETURN',
                                                    'aborted':'ANNOUNCE_RETURN'})

            smach.StateMachine.add('ANNOUNCE_RETURN',
                                   AnnounceState(self.announcer,
                                                 "Returning to search line"),
                                   transitions = {'next':'RETURN_TO_START'})

            #return to start along the approach point
            #if the path is ever blocked just give up and return to the level_two search
            smach.StateMachine.add('RETURN_TO_START',
                                   MoveToPoints(self.tf_listener),
                                   transitions = {'next_point':'RETURN_MOVE',
                                                  'sample_detected':'RETURN_MOVE',
                                                  'complete':'complete'},
                                   remapping = {'point_list':'approach_points',
                                                'face_next_point':'true',
                                                'obstacle_check':'true'})
            
            smach.StateMachine.add('RETURN_MOVE',
                                   ExecuteSimpleMove(self.simple_mover),
                                   transitions = {'complete':'RETURN_TO_START',
                                                  'blocked':'ANNOUNCE_RETURN_BLOCKED',
                                                  'timeout':'RETURN_TO_START',
                                                  'aborted':'PURSUE_SAMPLE_ABORTED'},
                                   remapping = {'velocity':'pursuit_velocity'})

            smach.StateMachine.add('ANNOUNCE_RETURN_BLOCKED',
                                    AnnounceState(self.announcer,
                                                  "Return blocked, continue ing from here"),
                                    transitions = {'next':'complete'})

            smach.StateMachine.add('PURSUE_SAMPLE_ABORTED',
                                   PursueSampleAborted(self.result_pub),
                                   transitions = {'next':'aborted'})

        #action server wrapper    
        pursue_sample_server = smach_ros.ActionServerWrapper(
            'pursue_sample', samplereturn_msg.GeneralExecutiveAction,
            wrapped_container = self.state_machine,
            succeeded_outcomes = ['complete'],
            preempted_outcomes = ['preempted'],
            aborted_outcomes = ['aborted'],
            goal_key = 'action_goal',
            result_key = 'action_result')
        
        #introspection server
        sls = smach_ros.IntrospectionServer('smach_grab_introspection',
                                            self.state_machine,
                                            '/START_SAMPLE_PURSUIT')
      
        #subscribers, need to go after state_machine
        rospy.Subscriber('detected_sample_search',
                        samplereturn_msg.NamedPoint,
                        self.sample_detection_search)
        
        rospy.Subscriber('detected_sample_manipulator',
                        samplereturn_msg.NamedPoint,
                        self.sample_detection_manipulator)
        
        rospy.Subscriber('costmap_check',
                                samplereturn_msg.CostmapCheck,
                                self.handle_costmap_check)
        
        rospy.Subscriber("pause_state", std_msg.Bool, self.pause_state_update)

        #start action servers and services
        sls.start()
        pursue_sample_server.run_server()
        rospy.spin()
        sls.stop()
        
    def check_for_stop(self):
        
        active_strafe_key = self.state_machine.userdata.active_strafe_key
        
        #stop if we are blocked in strafe direction
        if active_strafe_key is not None:
            if self.strafes[active_strafe_key]['blocked']:
                rospy.loginfo("PURSUE SAMPLE stopping simple_mover on strafe %s blocked" %(active_strafe_key))
                return True

            #stop if we are checking for min pursuit distance
            if self.state_machine.userdata.check_pursuit_distance:
                robot_point = util.get_current_robot_pose(self.tf_listener,
                        self.odometry_frame).pose.position
                sample_point = self.state_machine.userdata.target_sample.point
                distance_to_sample = util.point_distance_2d(robot_point, sample_point)
                self.state_machine.userdata.distance_to_sample = distance_to_sample
                if (distance_to_sample <= self.state_machine.userdata.min_pursuit_distance):
                    rospy.loginfo("PURSUE SAMPLE stopping simple_move on pursuit distance = %.3f" %(distance_to_sample))
                    return True


        #stop if we are moving and looking for manipulator detections
        if self.state_machine.userdata.stop_on_sample and \
           self.state_machine.userdata.detected_sample is not None:
            rospy.loginfo("PURSUE SAMPLE stopping simple_mover on manipulator detection")
            return True
                    
        return False
    
    def handle_costmap_check(self, costmap_check):
        for strafe_key, blocked in zip(costmap_check.strafe_keys,
                                       costmap_check.blocked):
            self.strafes[strafe_key]['blocked'] = blocked                        

    def sample_detection_search(self, sample):
            try:
                self.tf_listener.waitForTransform(self.odometry_frame,
                                                  sample.header.frame_id,
                                                  sample.header.stamp,
                                                  rospy.Duration(1.0))
                point_in_frame = self.tf_listener.transformPoint(self.odometry_frame, sample)
                sample.header = point_in_frame.header
                sample.point = point_in_frame.point
                self.state_machine.userdata.target_sample = sample
            except tf.Exception:
                rospy.logwarn("PURSUE_SAMPLE failed to transform search detection point!")

    def sample_detection_manipulator(self, sample):
        self.state_machine.userdata.detected_sample = sample
 
    def pause_state_update(self, msg):
        self.state_machine.userdata.paused = msg.data
            
    def shutdown_cb(self):
        self.state_machine.request_preempt()
        while self.state_machine.is_running():
            rospy.sleep(0.1)
    
#searches the globe   
class StartSamplePursuit(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['next'],
                             input_keys=['target_sample',
                                         'action_goal',
                                         'pursuit_point'],
                             output_keys=['velocity',
                                          'approach_points',
                                          'pursuit_point',
                                          'last_sample_detection_time',
                                          'stop_on_sample',
                                          'action_result',
                                          'search_count',
                                          'simple_move',
                                          'distance_to_sample'])
  
    
    def execute(self, userdata):
        
        result = samplereturn_msg.GeneralExecutiveResult()
        result.result_string = 'approach failed'
        result.result_int = samplereturn_msg.NamedPoint.NONE
        userdata.action_result = result
        #leave this false until we are in manipulator view
        userdata.stop_on_sample = False
        #default velocity for all moves is in simple_mover,
        #initial approach pursuit done at pursuit_velocity
        userdata.velocity = None
        userdata.search_count = 0
        rospy.loginfo("SAMPLE_PURSUIT input_point: %s" % (userdata.action_goal.input_point))
        userdata.pursuit_point = userdata.action_goal.input_point
        userdata.approach_points = collections.deque([])
        userdata.distance_to_sample = 20 #maximum possible detection
 
        return 'next'
 
class ApproachPoint(smach.State):
    def __init__(self, tf_listener, announcer):
        smach.State.__init__(self,
                             outcomes=['complete',
                                       'move',
                                       'point_lost',
                                       'blocked',
                                       'aborted'],
                             input_keys=['strafes',
                                         'target_sample',
                                         'approach_points',
                                         'offset_count',
                                         'active_strafe_key',
                                         'min_pursuit_distance',
                                         'pursuit_step',
                                         'odometry_frame',
                                         'distance_to_sample'],
                             output_keys=['simple_move',
                                          'approach_points',
                                          'offset_count',
                                          'active_strafe_key',
                                          'check_pursuit_distance'])
        
        self.tf_listener = tf_listener
        self.announcer = announcer
    
    def execute(self, userdata):
        
        #get all the relationships between robot and sample, angle, distance, etc.
        current_pose = util.get_current_robot_pose(self.tf_listener,
                userdata.odometry_frame)
        sample_point = userdata.target_sample.point
        robot_point = current_pose.pose.position
        yaw_to_sample = util.pointing_yaw(robot_point, sample_point)
        actual_yaw = util.get_current_robot_yaw(self.tf_listener,
                userdata.odometry_frame)
        rotate_yaw = util.unwind(yaw_to_sample - actual_yaw)
        
        #check_for_stop needs this flag
        userdata.check_pursuit_distance = False
 
        #this is the first move, which is always to point right at the sample
        if len(userdata.approach_points) == 0:
            self.announcer.say("Sample detected.")
            userdata.offset_count = 0
            userdata.approach_points.appendleft(robot_point)
            return self.spin(rotate_yaw, userdata)
        else:
            #last move was a spin, which is only done to point at the sample
            #time to strafe towards sample
            if userdata.active_strafe_key is None:
                return self.strafe('center', userdata)
            #getting here means last move was a strafe, first check if it was an offset move,
            #which would only be done if center blocked, if so, face the sample again, even if
            #the offset move was blocked, perhaps we got a view on the sample
            elif userdata.active_strafe_key != 'center':
                #append points at ends of offset moves
                userdata.approach_points.appendleft(robot_point)
                return self.spin(rotate_yaw, userdata)
            #last move was not a spin, nor offset, must have been center
            #was it blocked?
            elif userdata.strafes['center']['blocked']:
                #regardless, we got her from a center strafe, so append the point
                userdata.approach_points.appendleft(robot_point)
                #only one offset attempt allowed in sample pursuit, time to leave if it's not zero now
                if userdata.offset_count != 0:
                    userdata.active_strafe_key = None
                    self.announcer.say("Sample approach blocked. Abort ing")
                    return 'blocked'
                else:
                    #if left is clear strafe over there
                    if not userdata.strafes['left']['blocked']:
                        #append points at beginning of offsets
                        userdata.approach_points.appendleft(robot_point)
                        return self.strafe('left', userdata)
                    #if not, then right
                    elif not userdata.strafes['right']['blocked']:
                        #append points at beginning of offsets
                        userdata.approach_points.appendleft(robot_point)
                        return self.strafe('right', userdata)
                    #all blocked, get the hell out
                    else:
                        userdata.active_strafe_key = None
                        self.announcer.say("Sample approach blocked, abort ing")
                        return 'blocked'                    
            #center is clear, keep going
            else:
                return self.strafe('center', userdata)
            
            return 'aborted'
 
    def strafe(self, key, userdata):
        #check before any strafe move:
        if userdata.distance_to_sample <= userdata.min_pursuit_distance:
            rospy.loginfo("PURSUIT within minimum distance")
            userdata.active_strafe_key = None
            return 'complete'
        strafe = userdata.strafes[key]
        distance = strafe['distance']
        userdata.offset_count += np.sign(strafe['angle'])
        userdata.simple_move = {'type':'strafe',
                                'angle':strafe['angle'],
                                'distance': distance}
        #check_for_stop needs this flag
        userdata.check_pursuit_distance = True
        userdata.active_strafe_key = key
        if key == 'center':
            self.announcer.say("Close ing on sample")           
        elif key == 'left':
            self.announcer.say("Sample approach blocked, strafe ing left")
        elif key == 'right':
            self.announcer.say("Sample approach blocked, strafe ing right")
        return 'move'
    
    def spin(self, angle, userdata):
        userdata.simple_move = {'type':'spin',
                                'angle':angle}
        userdata.active_strafe_key = None
        self.announcer.say("Rotate ing towards sample")
        return 'move'

class GetManipulatorApproachMove(smach.State):
    def __init__(self, tf_listener):
        smach.State.__init__(self,
                             outcomes=['strafe', 'point_lost', 'aborted'],
                             input_keys=['target_sample',
                                         'settle_time'],
                             output_keys=['simple_move',
                                          'target_sample',
                                          'detected_sample',
                                          'stop_on_sample'])
        
        self.tf_listener = tf_listener
        
    def execute(self, userdata):
        
        userdata.target_sample = None
        rospy.sleep(userdata.settle_time)
        if userdata.target_sample is None:
            return 'point_lost'
        else:
            sample_time = userdata.target_sample.header.stamp
            sample_frame = userdata.target_sample.header.frame_id
            try:
                self.tf_listener.waitForTransform('base_link',
                        sample_frame, sample_time, rospy.Duration(1.0))
                point_in_base = self.tf_listener.transformPoint('base_link',
                                                             userdata.target_sample).point
            except(tf.Exception):
                rospy.logwarn("PURSUE_SAMPLE failed to get base_link -> %s transform in 1.0 seconds", sample_frame)
                return 'aborted'
            origin = geometry_msg.Point(0,0,0)
            distance = util.point_distance_2d(origin, point_in_base)
            yaw = util.pointing_yaw(origin, point_in_base)
            userdata.detected_sample = None
            #time to look for sample in manipulator view, stop when it is seen
            userdata.stop_on_sample = True
            userdata.simple_move = {'type':'strafe',
                                    'angle':yaw,
                                    'distance':distance}
            return 'strafe'
        
        return 'aborted'

class HandleSearch(smach.State):
    def __init__(self, tf_listener, announcer):
        smach.State.__init__(self,
                             outcomes=['sample_search', 'sample_detected', 'aborted'],
                             input_keys=['square_search_size',
                                         'search_count',
                                         'search_try_limit',
                                         'odometry_frame',
                                         'detected_sample',
                                        ],
                             output_keys=['point_list',
                                          'search_count'])
    
        self.tf_listener = tf_listener
        self.announcer = announcer

    def execute(self, userdata):

        if userdata.detected_sample is not None:
            self.announcer.say("Sample in manipulator view")
            return 'sample_detected'

        if userdata.search_count >= userdata.search_try_limit:
            self.announcer.say("Search limit reached")

        point_list = collections.deque([])
        square_step = userdata.square_search_size

        try:
            start_pose = util.get_current_robot_pose(self.tf_listener,
                    userdata.odometry_frame)
            next_pose = util.translate_base_link(self.tf_listener, start_pose, square_step, 0)
            point_list.append(next_pose.pose.position)
            next_pose = util.translate_base_link(self.tf_listener, start_pose, square_step, square_step)
            point_list.append(next_pose.pose.position)
            next_pose = util.translate_base_link(self.tf_listener, start_pose, -square_step, square_step)
            point_list.append(next_pose.pose.position)
            next_pose = util.translate_base_link(self.tf_listener, start_pose, -square_step, -square_step)
            point_list.append(next_pose.pose.position)
            next_pose = util.translate_base_link(self.tf_listener, start_pose, square_step, -square_step)
            point_list.append(next_pose.pose.position)
            userdata.point_list = point_list
            self.announcer.say("No sample detected. Search ing area")           
            userdata.search_count += 1
            return 'sample_search'
            
        except tf.Exception:
            rospy.logwarn("PURSUE_SAMPLE failed to transform robot pose in GetSearchMoves")
            return 'aborted'

class MoveToPoints(smach.State):
    def __init__(self, tf_listener):
        smach.State.__init__(self,
                             input_keys=['point_list',
                                         'detected_sample',
                                         'odometry_frame',
                                         'stop_on_sample',
                                         'face_next_point',
                                         'obstacle_check'],
                             output_keys=['simple_move',
                                          'point_list',
                                          'active_strafe_key'],
                             outcomes=['next_point',
                                       'sample_detected',
                                       'complete',
                                       'aborted'])
        
        self.tf_listener = tf_listener
        self.facing_next_point = False
            
    def execute(self, userdata):    
        
        #if we are looking for samples, mover will be stopped, check it here
        if userdata.detected_sample is not None and userdata.stop_on_sample:
            return 'sample_detected'
        
        if (len(userdata.point_list) > 0):

            target_point = userdata.point_list[0]
            header = std_msg.Header(0, rospy.Time(0), userdata.odometry_frame)
            search_point = geometry_msg.PointStamped(header, target_point)
            try:
                point_in_base = self.tf_listener.transformPoint('base_link', search_point).point                
            except(tf.Exception):
                rospy.logwarn("PURSUE_SAMPLE failed to transform search point (%s) to base_link in 1.0 seconds", search_point.header.frame_id)
                return 'aborted'

            origin = geometry_msg.Point(0,0,0)
            distance = util.point_distance_2d(origin, point_in_base)
            pointing_yaw = util.pointing_yaw(origin, point_in_base)

            if userdata.face_next_point and not self.facing_next_point:
                userdata.simple_move = {'type':'spin',
                                        'angle':pointing_yaw}
                userdata.active_strafe_key = None
                self.facing_next_point = True
                return 'next_point'
            else:
                #check obstacles along center if we are facing next point
                if userdata.obstacle_check and self.facing_next_point:
                    userdata.active_strafe_key = 'center'
                else:
                    userdata.active_strafe_key = None
                
                #remove the point if we are heading to it
                userdata.point_list.popleft()
                userdata.simple_move = {'type':'strafe',
                                        'angle':pointing_yaw,
                                        'distance':distance}
                
                self.facing_next_point = False
                return 'next_point'                
                    
            return 'next_point'
        else:
            self.facing_next_point = False
            return 'complete'
        
        return 'aborted'

class ConfirmSampleAcquired(smach.State):
    def __init__(self, announcer, result_pub):
        smach.State.__init__(self,
                             outcomes=['sample_gone',
                                       'sample_present',
                                       'aborted'],
                             input_keys=['latched_sample',
                                        'detected_sample',
                                        'action_result'],
                             output_keys=['detected_sample',
                                          'action_result'])

        self.announcer = announcer
        self.result_pub = result_pub
    
    def execute(self, userdata):
        
        #wait for 1 second, see if sample is present in view
        userdata.detected_sample = None
        rospy.sleep(1.0)
        if userdata.detected_sample is None:
            #this is the path of great success, notify filter nodes about success
            #and return the acquired sample ID in case the calling executive needs it
            self.announcer.say("Sample acquired")
            self.result_pub.publish(samplereturn_msg.PursuitResult(True))
            userdata.action_result.result_string = ('sample acquired')
            userdata.action_result.result_int = userdata.latched_sample.sample_id
            return 'sample_gone'
        else:
            if userdata.detected_sample.sample_id == userdata.latched_sample.sample_id:
                self.announcer.say("Sample acquisition failed, retry ing")
                return 'sample_present'
            else:
                self.announcer.say("New sample in view, confuse ing")
                return 'sample_gone'

class SearchLightSwitch(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['next'],
                             input_keys =['on'])
        
    def execute(self, userdata):
        pass

class PublishFailure(smach.State):
    def __init__(self, result_pub):
        smach.State.__init__(self, outcomes=['next'])
        self.result_pub = result_pub
    def execute(self, userdata):
        self.result_pub.publish(samplereturn_msg.PursuitResult(False))        
        return 'next'

class PursueSampleAborted(smach.State):
    def __init__(self, result_pub):
        smach.State.__init__(self, outcomes=['next'])
        self.result_pub = result_pub
    def execute(self, userdata):
        self.result_pub.publish(samplereturn_msg.PursuitResult(False))
        return 'next'
