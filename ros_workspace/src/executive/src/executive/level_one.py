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

from executive.executive_states import DriveToPoseState
from executive.executive_states import PursueDetectedPoint


import samplereturn.util as util

class LevelOne(object):    

    def __init__(self):
        
        self.announcer = util.AnnouncerInterface("audio_navigate")
        self.tf_listener = tf.TransformListener()
        
        self.node_params = util.get_node_params()

        self.move_base = actionlib.SimpleActionClient("planner_move_base",
                                                       move_base_msg.MoveBaseAction)
        
        start_time = rospy.get_time()
        while True:
            if self.move_base.wait_for_server(rospy.Duration(0.001)):
                break #all services up, exit this loop
            if (rospy.get_time() - start_time) > 10.0:
                self.announcer.say("Move base not available")
                rospy.logwarn('Timeout waiting for move base')
            rospy.sleep(0.1)

        self.state_machine = smach.StateMachine(
                outcomes=['complete', 'preempted', 'aborted'],
                input_keys = ['action_goal'],
                output_keys = ['action_result'])

        self.state_machine.userdata.max_pursuit_error = 0.2        
        self.state_machine.userdata.min_pursuit_distance = 3
        self.state_machine.userdata.max_point_lost_time = 30
        
        with self.state_machine:
            
                smach.StateMachine.add('START_LEVEL_ONE',
                                       StartLevelOne(self.announcer),
                                       transitions = {'next':'DRIVE_TO_SEARCH_START'})
                
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
                                       transitions = {'complete':'RETURN_HOME',
                                                      'timeout':'RETURN_HOME',
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
                                      transitions = {'succeeded':'RETURN_HOME',
                                                     'preempted':'PURSUE_SAMPLE',
                                                     'aborted':'LEVEL_ONE_ABORTED'})               
               
                smach.StateMachine.add('RETURN_HOME',
                                       ReturnHome(self.announcer),
                                       transitions = {'next':'APPROACH_BEACON'})
                
                smach.StateMachine.add('APPROACH_BEACON',
                       PursueDetectedPoint(self.announcer,
                                           self.move_base,
                                           self.tf_listener,
                                           within_min_msg = 'Reverse ing on to platform'),
                       transitions = {'min_distance':'MOUNT_PLATFORM',
                                      'point_lost':'RETURN_HOME'},
                       remapping = {'target_point':'beacon_target'})

                smach.StateMachine.add('MOUNT_PLATFORM',
                                       DriveToPoseState(self.move_base,
                                                        self.tf_listener),
                                       transitions = {'complete':'complete',
                                                      'timeout':'RETURN_HOME',
                                                      'sample_detected':'LEVEL_ONE_ABORTED'})
                
                smach.StateMachine.add('LEVEL_ONE_PREEMPTED',
                                       LevelOnePreempted(),
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

        #start action servers and services
        sls.start()
        level_one_server.run_server()
        
        #subscribers, need to go after state_machine
        self.sample_listener = rospy.Subscriber('detected_sample_search',
                                        samplereturn_msg.NamedPoint,
                                        self.sample_update)
        
        self.beacon_listener = rospy.Subscriber("beacon_pose",
                                                geometry_msg.PoseStamped,
                                                self.beacon_update)
        
    def sample_update(self, sample):
        if sample.name == 'none':
            self.state_machine.userdata.detected_sample = None
        else:
            self.state_machine.userdata.detected_sample = sample
            
    def beacon_update(self, beacon_pose):
        header = beacon_pose.header
        point = beacon_pose.pose.position
        beacon_target = geometry_msg.PointStamped(header, point)
        self.state_machine.userdata.beacon_target = beacon_target
    
#drives to the expected sample location    
class StartLevelOne(smach.State):
    def __init__(self, announcer):
        smach.State.__init__(self,
                             output_keys=['action_result',
                                         'pose_list'],
                             outcomes=['next'])
        
        self.announcer = announcer
        
    def execute(self, userdata):
        
        pose_2d_list = [{'x':3, 'y':0, 'heading':45},
                        {'x':13, 'y':10, 'heading':30},
                        {'x':20, 'y':15, 'heading':0}]

        header = std_msg.Header(0, rospy.Time(0), '/map')
        path_pose_list = []
        for pose_2d in pose_2d_list:
            pose = geometry_msg.Pose()
            pose.position = geometry_msg.Point(pose_2d['x'],
                                               pose_2d['y'],
                                               0.0)
            quat_array = tf.transformations.quaternion_from_euler(0, 0, pose_2d['heading'])           
            pose.orientation = geometry_msg.Quaternion(*quat_array)
            pose_stamped = geometry_msg.PoseStamped(header, pose)
            path_pose_list.append(pose_stamped)

        #put the expected sample location on the end of the path list
        #same orientation as the robot's last pose
        sample_position = geometry_msg.Point(35, 10, 0)
        pose = geometry_msg.Pose()
        pose.position = sample_position
        pose.orientation = path_pose_list[-1].pose.orientation
        pose_stamped = geometry_msg.PoseStamped(header, pose)
        path_pose_list.append(pose_stamped)
        
        userdata.pose_list = path_pose_list        
        
        result = samplereturn_msg.GeneralExecutiveResult('initialized')
        userdata.action_result = result
        
        self.announcer.say("Entering level one mode")
        rospy.sleep(3.0)
        
        return 'next'

class DriveToSearch(smach.State):
    def __init__(self, announcer):
        smach.State.__init__(self,
                             input_keys=['pose_list'],
                             output_keys=['target_pose','velocity','pursue_samples', 'pose_list', 'detected_sample'],
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

   
class ReturnHome(smach.State):
 
    def __init__(self, announcer):

        smach.State.__init__(self,
                             outcomes=['next',
                                       'preempted',
                                       'aborted'],
                             output_keys=['beacon_target',
                                          'pursue_samples'])
        
        self.announcer = announcer

    def execute(self, userdata):
        
        self.announcer.say("Return ing to platform")

        header = std_msg.Header(0, rospy.Time(0), '/map')
        #the beacon is probably not in view, drive to a point 15m in front of it
        point = geometry_msg.PointStamped(header, geometry_msg.Point(15, 0, 0))
        userdata.beacon_target = point
        userdata.pursue_samples = False
        
        return 'next'

class ApproachBeacon(smach.State):
    def __init__(self, announcer, move_client, listener):
        smach.State.__init__(self,
                             outcomes=['close_enough', 'beacon_lost'],
                             input_keys=['beacon_target',
                                       'beacon_close_distance',
                                       'max_pursuit_error',
                                       'max_beacon_lost_time'],
                             output_keys=['beacon_target',
                                          'target_pose',
                                          'velocity',
                                          'pursue_samples'])
    
        self.announcer = announcer
        self.move_client = move_client
        self.listener = listener

    def execute(self, userdata):
        
        move_goal = move_base_msg.MoveBaseGoal()
        move_goal.target_pose = userdata.beacon_target 
        self.move_client.send_goal(move_goal)
        
        rospy.loginfo("PURSUE BEACON initial pursuit point: %s", userdata.beacon_target)

        last_beacon_detection = rospy.get_time()

        while True:
            current_pose= util.get_current_robot_pose(self.listener)
            move_state = self.move_client.get_state()
            if move_state not in util.actionlib_working_states:
                return 'aborted'
            
            beacon_distance = util.pose_distance_2d(current_pose, move_goal.target_pose)
            rospy.loginfo("APPROACH DISTANCE: " + str(beacon_distance))
            if beacon_distance < userdata.beacon_close_distance:
                userdata.velocity = 0.5
                userdata.pursue_samples = False
                userdata.target_pose = move_goal.target_pose
                self.announcer.say("Reverse ing on to platform")
                return 'close_enough'
            
            if userdata.beacon_target is not None:
                last_beacon_detection = rospy.get_time()
                beacon_target_point = geometry_msg.PointStamped()
                beacon_target_point.header = userdata.beacon_target.header
                beacon_target_point.point = userdata.beacon_target.pose.position
                beacon_on_map = self.listener.transformPoint('/map', beacon_target_point)
                
                current_point = move_goal.target_pose.pose.position
                beacon_distance_delta = util.point_distance_2d(current_point, beacon_on_map.point)
                if  beacon_distance_delta > userdata.max_pursuit_error:
                    rospy.loginfo("PURSUE BEACON updated pursuit point %s", beacon_on_map)
                    
                    move_goal.target_pose.pose.position = beacon_on_map.point
                    self.move_client.send_goal(move_goal)
            
            if (rospy.get_time() - last_beacon_detection) > userdata.max_beacon_lost_time:
                return 'beacon_lost'
                            
            userdata.beacon_target = None       
            rospy.sleep(0.2)
    
    
class LevelOnePreempted(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             output_keys=['action_result'],
                             outcomes=['complete','fail'])
          
    def execute(self, userdata):
        
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