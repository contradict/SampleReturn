#!/usr/bin/env python
import math
import smach
import smach_ros
import rospy
import collections
import threading
import actionlib
import tf

import std_msgs.msg as std_msg
import visual_servo_msgs.msg as visual_servo_msg
import samplereturn_msgs.msg as samplereturn_msg
import move_base_msgs.msg as move_base_msg
import geometry_msgs.msg as geometry_msg
import platform_motion_msgs.msg as platform_msg
import platform_motion_msgs.srv as platform_srv

from executive.executive_states import DriveToPoseState
from executive.executive_states import PursueDetectedPoint
from executive.executive_states import SelectMotionMode
from executive.executive_states import AnnounceState

import samplereturn.util as util

class LevelOne(object):    

    def __init__(self):
        
        rospy.on_shutdown(self.shutdown_cb)
        
        self.announcer = util.AnnouncerInterface("audio_navigate")
        self.tf_listener = tf.TransformListener()
        
        self.node_params = util.get_node_params()

        self.move_base = actionlib.SimpleActionClient("planner_move_base",
                                                       move_base_msg.MoveBaseAction)
        self.CAN_interface = util.CANInterface()
        
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            start_time =  rospy.get_time()
            if self.move_base.wait_for_server(rospy.Duration(0.1)):
                break #all services up, exit this loop
            if (rospy.get_time() - start_time) > 10.0:
                self.announcer.say("Move base not available")
                rospy.logwarn('Timeout waiting for move base')
            rospy.sleep(0.1)

        self.state_machine = smach.StateMachine(
                outcomes=['complete', 'preempted', 'aborted'],
                input_keys = ['action_goal'],
                output_keys = ['action_result'])

        self.state_machine.userdata.max_pursuit_error = self.node_params.max_pursuit_error       
        self.state_machine.userdata.min_pursuit_distance = self.node_params.min_pursuit_distance
        self.state_machine.userdata.max_point_lost_time = self.node_params.max_point_lost_time
        self.state_machine.userdata.motion_check_interval = self.node_params.motion_check_interval
        self.state_machine.userdata.min_motion = self.node_params.min_motion        
        self.state_machine.userdata.search_poses_2d = self.node_params.search_poses_2d
        self.state_machine.userdata.beacon_approach_point = self.node_params.beacon_approach_point
        self.state_machine.userdata.detected_sample = None
        
        with self.state_machine:
            
                smach.StateMachine.add('START_LEVEL_ONE',
                                       StartLevelOne(output_keys=['action_result',
                                                                  'stop_on_sample',
                                                                  'pose_list'],
                                                     input_keys=['search_poses_2d'],
                                                     outcomes=['next']),
                                       transitions = {'next':'ANNOUNCE_LEVEL_ONE'})
                                       
                smach.StateMachine.add('ANNOUNCE_LEVEL_ONE',
                                       AnnounceState(self.announcer,
                                                     'Enter ing level one mode'),
                                       transitions = {'next':'SELECT_PLANNER'})
                
                MODE_PLANNER = platform_srv.SelectMotionModeRequest.MODE_PLANNER_TWIST
                smach.StateMachine.add('SELECT_PLANNER',
                                        SelectMotionMode(self.CAN_interface,
                                                         self.announcer,
                                                         MODE_PLANNER),
                                        transitions = {'next':'DRIVE_TO_SEARCH_START',
                                                      'failed':'LEVEL_ONE_ABORTED'})                
                
                smach.StateMachine.add('DRIVE_TO_SEARCH_START',
                                       DriveToSearch(self.announcer),
                                       transitions = {'next_point':'DRIVE_TO_POSE',
                                                      'complete':'SEARCH_FOR_SAMPLE',
                                                      'preempted':'LEVEL_ONE_PREEMPTED'})
                
                smach.StateMachine.add('DRIVE_TO_POSE',
                                       DriveToPoseState(self.move_base, self.tf_listener),
                                       transitions = {'complete':'DRIVE_TO_SEARCH_START',
                                                      'timeout':'DRIVE_TO_SEARCH_START',
                                                      'sample_detected':'PURSUE_SAMPLE'})
                
                smach.StateMachine.add('SEARCH_FOR_SAMPLE',
                                       DriveToPoseState(self.move_base, self.tf_listener),
                                       transitions = {'complete':'START_RETURN_HOME',
                                                      'timeout':'START_RETURN_HOME',
                                                      'sample_detected':'PURSUE_SAMPLE'})
                
                
                @smach.cb_interface(input_keys=['detected_sample'])
                def get_pursuit_goal_cb(userdata, request):
                    goal = samplereturn_msg.GeneralExecutiveGoal()
                    goal.name = userdata.detected_sample.name
                    return goal
                                
                smach.StateMachine.add('PURSUE_SAMPLE',
                                      smach_ros.SimpleActionState('pursue_sample',
                                      samplereturn_msg.GeneralExecutiveAction,
                                      goal_cb = get_pursuit_goal_cb),
                                      transitions = {'succeeded':'START_RETURN_HOME',
                                                     'preempted':'PURSUE_SAMPLE',
                                                     'aborted':'LEVEL_ONE_ABORTED'})               
               
                smach.StateMachine.add('START_RETURN_HOME',
                                       StartReturnHome(self.announcer),
                                       transitions = {'next':'DRIVE_TO_BEACON_APPROACH_START'})
                
                smach.StateMachine.add('DRIVE_TO_BEACON_APPROACH_START',
                                       DriveToPoseState(self.move_base,
                                                        self.tf_listener),
                                       transitions = {'complete':'APPROACH_BEACON',
                                                      'timeout':'START_RETURN_HOME',
                                                      'sample_detected':'LEVEL_ONE_ABORTED'})
                
                smach.StateMachine.add('APPROACH_BEACON',
                       PursueDetectedPoint(self.announcer,
                                           self.move_base,
                                           self.tf_listener,
                                           within_min_msg = 'Reverse ing on to platform'),
                       transitions = {'min_distance':'MOUNT_PLATFORM',
                                      'point_lost':'START_RETURN_HOME'},
                       remapping = {'target_point':'beacon_target'})

                smach.StateMachine.add('MOUNT_PLATFORM',
                                       DriveToPoseState(self.move_base,
                                                        self.tf_listener),
                                       transitions = {'complete':'DESELECT_PLANNER',
                                                      'timeout':'START_RETURN_HOME',
                                                      'sample_detected':'LEVEL_ONE_ABORTED'})

                MODE_ENABLE = platform_srv.SelectMotionModeRequest.MODE_ENABLE
                smach.StateMachine.add('DESELECT_PLANNER',
                                        SelectMotionMode(self.CAN_interface,
                                                         self.announcer,
                                                         MODE_ENABLE),
                                        transitions = {'next':'complete',
                                                      'failed':'LEVEL_ONE_ABORTED'})    
                
                smach.StateMachine.add('LEVEL_ONE_PREEMPTED',
                                       LevelOnePreempted(self.CAN_interface),
                                       transitions = {'complete':'preempted',
                                                      'fail':'aborted'})
                
                smach.StateMachine.add('LEVEL_ONE_ABORTED',
                                       LevelOneAborted(),
                                       transitions = {'recover':'SEARCH_FOR_SAMPLE',
                                                      'fail':'aborted'})

        #action server wrapper    
        level_one_server = smach_ros.ActionServerWrapper(
            'level_one', samplereturn_msg.GeneralExecutiveAction,
            wrapped_container = self.state_machine,
            succeeded_outcomes = ['complete'],
            preempted_outcomes = ['preempted'],
            aborted_outcomes = ['aborted'],
            goal_key = 'action_goal',
            result_key = 'action_result')
        
        #introspection server
        sls = smach_ros.IntrospectionServer('smach_grab_introspection',
                                            self.state_machine,
                                            '/START_LEVEL_ONE')
       
        #subscribers, need to go after state_machine
        self.sample_listener = rospy.Subscriber('detected_sample_search',
                                        samplereturn_msg.NamedPoint,
                                        self.sample_update)
        
        self.beacon_listener = rospy.Subscriber("beacon_pose",
                                                geometry_msg.PoseStamped,
                                                self.beacon_update)

        #start action servers and services
        sls.start()
        level_one_server.run_server()
        rospy.spin()
        sls.stop()
        
    def sample_update(self, sample):
        if sample.name == 'none':
            self.state_machine.userdata.detected_sample = None
        else:
            self.state_machine.userdata.detected_sample = sample
            
    def beacon_update(self, beacon_pose):
        beacon_pose.header.stamp = rospy.Time(0)
        map_beacon_pose = self.tf_listener.transformPose('/map', beacon_pose)
        header = map_beacon_pose.header
        point = map_beacon_pose.pose.position
        beacon_target = geometry_msg.PointStamped(header, point)
        self.state_machine.userdata.beacon_target = beacon_target
        
    def shutdown_cb(self):
        self.state_machine.request_preempt()
        while self.state_machine.is_running():
            rospy.sleep(0.1)
    
#drives to the expected sample location    
class StartLevelOne(smach.State):

    def execute(self, userdata):
        
        #the expected sample location on the end of the path list
        pose_2d_list = userdata.search_poses_2d

        header = std_msg.Header(0, rospy.Time(0), '/map')
        path_pose_list = []
        yaw = 0
        for i in range(len(pose_2d_list)):
            pose = geometry_msg.Pose()
            pose.position = geometry_msg.Point(pose_2d_list[i]['x'],
                                               pose_2d_list[i]['y'],
                                               0.0)
            if 'yaw' in pose_2d_list[i]:
                yaw = math.radians(pose_2d_list[i]['yaw'])
            else:
                if (i+1) < len(pose_2d_list):
                    next_point = geometry_msg.Point(pose_2d_list[i + 1]['x'],
                                                    pose_2d_list[i + 1]['y'],
                                                    0.0)
                    yaw = util.pointing_yaw(pose.position, next_point)
                # if this is last point, just maintain current yaw                
                            
            quat_array = tf.transformations.quaternion_from_euler(0, 0, yaw)           
            pose.orientation = geometry_msg.Quaternion(*quat_array)
            pose_stamped = geometry_msg.PoseStamped(header, pose)
            path_pose_list.append(pose_stamped)
        
        userdata.pose_list = path_pose_list        
        userdata.stop_on_sample = False
        
        result = samplereturn_msg.GeneralExecutiveResult('initialized')
        userdata.action_result = result
        
        return 'next'

class DriveToSearch(smach.State):
    def __init__(self, announcer):
        smach.State.__init__(self,
                             input_keys=['pose_list'],
                             output_keys=['target_pose',
                                          'velocity',
                                          'pursue_samples',
                                          'pose_list',
                                          'detected_sample'],
                             outcomes=['next_point',
                                       'complete',
                                       'preempted',
                                       'aborted'])
        
        self.announcer = announcer
        
    def execute(self, userdata):

        if (len(userdata.pose_list) > 1):
            self.announcer.say("Driving to next point")
            userdata.target_pose = userdata.pose_list.pop(0)
            userdata.velocity = 1.0
            userdata.pursue_samples = False
            return 'next_point'
        else:
            self.announcer.say("Searching for sample")
            userdata.target_pose = userdata.pose_list.pop(0)
            userdata.velocity = 0.5
            userdata.detected_sample = None #clear any previous samples
            userdata.pursue_samples = True
            return 'complete'

   
class StartReturnHome(smach.State):
 
    def __init__(self, announcer):

        smach.State.__init__(self,
                             outcomes=['next',
                                       'preempted',
                                       'aborted'],
                             input_keys=['beacon_approach_point'],
                             output_keys=['target_pose',
                                          'velocity',
                                          'pursue_samples'])
        
        self.announcer = announcer

    def execute(self, userdata):
        
        self.announcer.say("Return ing to platform")

        header = std_msg.Header(0, rospy.Time(0), '/map')
        #the beacon is probably not in view, drive to a point probably in front of it
        approach_point = userdata.beacon_approach_point
        point = geometry_msg.Point(approach_point['x'],
                                   approach_point['y'],
                                   math.radians(approach_point['yaw']))
        quat_array = tf.transformations.quaternion_from_euler(0, 0, math.pi)           
        pose = geometry_msg.Pose(point, geometry_msg.Quaternion(*quat_array))
                
        userdata.target_pose = geometry_msg.PoseStamped(header, pose)
        userdata.pursue_samples = False
        
        return 'next'
   
class LevelOnePreempted(smach.State):
    def __init__(self, CAN_interface):
        smach.State.__init__(self,
                             output_keys=['action_result'],
                             outcomes=['complete','fail'])
        
        self.CAN_interface = CAN_interface  
          
    def execute(self, userdata):
        
        self.CAN_interface.select_mode(
            platform_srv.SelectMotionModeRequest.MODE_ENABLE )
        self.CAN_interface.publish_zero()        
        
        result = samplereturn_msg.GeneralExecutiveResult('preempted')
        userdata.action_result = result
                
        #shutdown all internal processes to level_one
        
        return 'complete'

class LevelOneAborted(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             output_keys=['action_result'],
                             outcomes=['recover','fail'])
        
    def execute(self, userdata):
        
        result = samplereturn_msg.GeneralExecutiveResult('aborted')
        userdata.action_result = result
        
        return 'fail'