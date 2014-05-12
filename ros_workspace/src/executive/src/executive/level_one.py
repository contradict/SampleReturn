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
                                       transitions = {'complete':'complete',
                                                      'timeout':'START_RETURN_HOME',
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
        beacon_pose.header.stamp = rospy.Time(0)
        map_beacon_pose = self.tf_listener.transformPose('/map', beacon_pose)
        header = map_beacon_pose.header
        point = map_beacon_pose.pose.position
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
        
        pose_2d_list = [{'x':3, 'y':0, 'yaw':math.radians(45)},
                        {'x':13, 'y':10, 'yaw':math.radians(30)},
                        {'x':20, 'y':15, 'yaw':math.radians(0)}]

        header = std_msg.Header(0, rospy.Time(0), '/map')
        path_pose_list = []
        for pose_2d in pose_2d_list:
            pose = geometry_msg.Pose()
            pose.position = geometry_msg.Point(pose_2d['x'],
                                               pose_2d['y'],
                                               0.0)
            quat_array = tf.transformations.quaternion_from_euler(0, 0, pose_2d['yaw'])           
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

   
class StartReturnHome(smach.State):
 
    def __init__(self, announcer):

        smach.State.__init__(self,
                             outcomes=['next',
                                       'preempted',
                                       'aborted'],
                             output_keys=['target_pose',
                                          'velocity',
                                          'pursue_samples'])
        
        self.announcer = announcer

    def execute(self, userdata):
        
        self.announcer.say("Return ing to platform")

        header = std_msg.Header(0, rospy.Time(0), '/map')
        #the beacon is probably not in view, drive to a point 15m in front of it
        point = geometry_msg.Point(15, 0, 0)
        quat_array = tf.transformations.quaternion_from_euler(0, 0, math.pi)           
        pose = geometry_msg.Pose(point, geometry_msg.Quaternion(*quat_array))
                
        userdata.target_pose = geometry_msg.PoseStamped(header, pose)
        userdata.pursue_samples = False
        
        return 'next'
   
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