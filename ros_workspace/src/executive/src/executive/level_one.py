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
        
        with self.state_machine:
            
                smach.StateMachine.add('START_LEVEL_ONE',
                                       StartLevelOne(self.announcer),
                                       transitions = {'next':'DRIVE_TO_SEARCH'})
                
                smach.StateMachine.add('DRIVE_TO_SEARCH',
                                       DriveToSearch(self.announcer),
                                       transitions = {'next_point':'DRIVE_TO_POSE',
                                                      'complete':'DRIVE_TO_SAMPLE',
                                                      'preempted':'LEVEL_ONE_PREEMPTED'})
                
                smach.StateMachine.add('DRIVE_TO_POSE',
                                       DriveToPoseState(self.move_base, self.tf_listener),
                                       transitions = {'complete':'DRIVE_TO_SEARCH',
                                                      'timeout':'DRIVE_TO_SEARCH',
                                                      'sample_detected':'PURSUE_SAMPLE'})
                
                smach.StateMachine.add('DRIVE_TO_SAMPLE',
                                       DriveToPoseState(self.move_base, self.tf_listener),
                                       transitions = {'complete':'RETURN_HOME',
                                                      'timeout':'RETURN_HOME',
                                                      'sample_detected':'PURSUE_SAMPLE'})
                
                smach.StateMachine.add('PURSUE_SAMPLE',
                                       PursueSample(self.announcer),
                                       transitions = {'sample_acquired':'RETURN_HOME'})
                
                smach.StateMachine.add('RETURN_HOME',
                                       ReturnHome(outcomes=['complete',
                                                            'preempted']),
                                       transitions = {'complete':'complete'})
                
                smach.StateMachine.add('LEVEL_ONE_PREEMPTED',
                                      LevelOnePreempted(),
                                       transitions = {'complete':'preempted',
                                                      'fail':'aborted'})
                
                smach.StateMachine.add('LEVEL_ONE_ABORTED',
                                       LevelOneAborted(),
                                       transitions = {'recover':'DRIVE_TO_SAMPLE',
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
    
#drives to the expected sample location    
class StartLevelOne(smach.State):
    def __init__(self, announcer):
        smach.State.__init__(self,
                             output_keys=['action_result', 'pose_list'],
                             outcomes=['next'])
        
        self.announcer = announcer
        
    def execute(self, userdata):
        
        pose_2d_list = [{'x':2, 'y':0, 'heading':-30},
                        {'x':20, 'y':35, 'heading':26},
                        {'x':25, 'y':30, 'heading':26}]

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
        sample_position = geometry_msg.Point(35, 25, 0)
        pose = geometry_msg.Pose()
        pose.position = sample_position
        pose.orientation = path_pose_list[-1].pose.orientation
        pose_stamped = geometry_msg.PoseStamped(header, pose)
        path_pose_list.append(pose_stamped)
        
        userdata.pose_list = path_pose_list        
        
        result = samplereturn_msg.GeneralExecutiveResult.COMPLETE
        userdata.action_result = result
        
        self.announcer.say("Entering level one mode")
        rospy.sleep(3.0)
        
        return 'next'

class DriveToSearch(smach.State):
    def __init__(self, announcer):
        smach.State.__init__(self,
                             input_keys=['current_pose', 'pose_list'],
                             output_keys=['target_pose','velocity','pursue_samples', 'pose_list'],
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
            userdata.pursue_samples = True
            return 'complete'
            

#drive to detected sample location        
class PursueSample(smach.State):
    def __init__(self, announcer):
        smach.State.__init__(self,
                             outcomes=['sample_acquired'])
        
        self.announcer = announcer
        
    def execute(self, userdata):
        
        self.announcer.say("Sample detected, pursue ing")
        rospy.sleep(5.0)
        self.announcer.say("Sample acquired")
        
            
        return 'sample_acquired'
    
class ReturnHome(smach.State):
 
    def execute(self, userdata):
        
        return 'complete'
    
    
class LevelOnePreempted(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             output_keys=['action_result'],
                             outcomes=['complete','fail'])
          
    def execute(self):
        
        result = samplereturn_msg.GeneralExecutiveResult.PREEMPTED
        userdata.action_result = result
                
        #shutdown all internal processes to level_one
        
        return 'complete'

class LevelOneAborted(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             output_keys=['action_result'],
                             outcomes=['recover','fail'])
        
    def execute(self):
        
        result = samplereturn_msg.GeneralExecutiveResult.PREEMPTED
        userdata.action_result = result
        
        return 'fail'