#!/usr/bin/env pyth
import sys
import math
import rospy
import rosnode
import actionlib
import tf
import tf_conversions

import smach
import smach_ros

import actionlib_msgs.msg as action_msg
import std_msgs.msg as std_msg
import sensor_msgs.msg as sensor_msgs
import platform_motion_msgs.msg as platform_msg
import platform_motion_msgs.srv as platform_srv
import manipulator_msgs.msg as manipulator_msg
import geometry_msgs.msg as geometry_msg
import move_base_msgs.msg as move_base_msg
import visual_servo_msgs.msg as visual_servo_msg
import linemod_detector.msg as linemod_msg
import samplereturn_msgs.msg as samplereturn_msg

import dynamic_reconfigure.srv as dynsrv
import dynamic_reconfigure.msg as dynmsg

from visual_servo_msgs.msg import VisualServoAction, VisualServoResult, VisualServoFeedback, TunableConstants

class RobotSimulator(object):
    
    def __init__(self):
    
        rospy.init_node("robot_simulator")
        
        #simulator state variables and constants            
        self.GPIO_LEVEL_ONE = 0x20
        self.GPIO_LEVEL_TWO = 0x08
        
        self.mode = 'level_one'
        self.cameras_ready = True
        self.paused = False
        self.publish_samples = True
        self.fake_sample = {'x':35, 'y':25, 'name':'PRECACHED'}
    
        #topic and action names
        manipulator_action_name = "/motion/manipulator/grab_action"
        home_wheelpods_name = "/motion/wheel_pods/home"
        home_carousel_name = "/motion/carousel/home"
        enable_wheelpods_name = "/motion/wheel_pods/enable"
        enable_carousel_name = "/motion/carousel/enable"
        
        select_motion_name = "/motion/CAN/select_motion_mode"

        cam_status_list = ["/cameras/navigation/port/status",
                                "/cameras/navigation/center/status",
                                "/cameras/navigation/starboard/status",
                                "/cameras/manipulator/status",
                                "/cameras/search/status"]
        
        gpio_read_name = "/io/gpio_read"
        pause_state_name = "/io/pause_state"
        
        move_base_name = "/processes/planner/move_base"
        
        visual_servo_name = "/processes/visual_servo/servo_action"
        
        detected_sample_name = "/processes/sample_detection/detected_samples"
   
        #manipulator stuff
        self.manipulator_sm = smach.StateMachine(
            outcomes=['success', 'aborted', 'preempted'],
            input_keys = ['action_goal'],
            output_keys = ['action_result'])
        
        with self.manipulator_sm:

            smach.StateMachine.add('START_MANIPULATOR',
                                    ManipulatorState(outcomes=['succeeded','preempted'],
                                                    input_keys=['action_goal','action_result'],
                                                    output_keys=['action_goal','action_result']),
                                    transitions = {'preempted': 'success',
                                                   'succeeded': 'success'})
            
        manipulator_action_server = smach_ros.ActionServerWrapper(
            manipulator_action_name,
            manipulator_msg.ManipulatorAction,
            wrapped_container = self.manipulator_sm,
            succeeded_outcomes = ['success'],
            aborted_outcomes = ['aborted'],
            preempted_outcomes = ['preempted'],
            goal_key = 'action_goal',
            result_key = 'action_result')
        manipulator_action_server.run_server()
       
        #camera publishers
        self.cam_publishers = []
        for topic in cam_status_list:
            self.cam_publishers.append(rospy.Publisher(topic, std_msg.String))
        rospy.Timer(rospy.Duration(0.5), self.publish_cam_status)
                
        #io publishers
        self.GPIO_pub = rospy.Publisher(gpio_read_name, platform_msg.GPIO)
        rospy.Timer(rospy.Duration(0.2), self.publish_GPIO)
        self.pause_pub = rospy.Publisher(pause_state_name, std_msg.Bool)
        rospy.Timer(rospy.Duration(0.2), self.publish_pause)


        #platform motion publishers, services, and action servers
        self.select_motion_mode = rospy.Service(select_motion_name,
                                                platform_srv.SelectMotionMode,
                                                self.service_motion_mode_request)
        self.enable_wheelpods = rospy.Service(enable_wheelpods_name,
                                              platform_srv.Enable,
                                              self.service_enable_wheelpods)
        self.enable_carousel = rospy.Service(enable_carousel_name,
                                             platform_srv.Enable,
                                             self.service_enable_wheelpods)
        
        self.home_wheelpods_server = actionlib.SimpleActionServer(home_wheelpods_name,
                                                                  platform_msg.HomeAction,
                                                                  self.home_wheelpods,
                                                                  False)
        self.home_carousel_server = actionlib.SimpleActionServer(home_carousel_name,
                                                                  platform_msg.HomeAction,
                                                                  self.home_carousel,
                                                                  False)
        
        self.home_carousel_server.start()
        self.home_wheelpods_server.start()

        #planner
        self.move_base_server = actionlib.SimpleActionServer(move_base_name,
                                                             move_base_msg.MoveBaseAction,
                                                             self.move_base,
                                                             False)
        
        self.move_base_server.start()
        
        #sample detection stuff
        self.sample_pub = rospy.Publisher(detected_sample_name, samplereturn_msg.NamedPoint)
        rospy.Timer(rospy.Duration(3.0), self.publish_sample_detection)        
        
        
                
        #visual servo stuff
        self.visual_servo_server = actionlib.SimpleActionServer(visual_servo_name,
                                                           VisualServoAction,
                                                           self.run_visual_servo_action,
                                                           False)
        self.visual_servo_server.start()        

        #rospy.spin()

    def publish_cam_status(self, event):
        if self.cameras_ready:
            msg = 'Ready'
        else:
            msg = 'Error'
        
        for pub in self.cam_publishers:
            pub.publish(std_msg.String(msg))        

    def publish_sample_detection(self, event):
        if self.publish_samples:
            msg = samplereturn_msg.NamedPoint()
            msg.header = header = std_msg.Header(0, rospy.Time(0), '/map')
            msg.point = geometry_msg.Point(self.fake_sample['x'],
                                                   self.fake_sample['y'],
                                                   0.0)
            msg.name = self.fake_sample['name']
            self.sample_pub.publish(msg)        

    def publish_GPIO(self, event):
        msg = platform_msg.GPIO()
        msg.servo_id = 1
        if self.mode == 'level_one':
            msg.new_pin_states = ~self.GPIO_LEVEL_ONE
        elif self.mode =='level_two':
            msg.new_pin_states = ~self.GPIO_LEVEL_TWO
        else:
            msg.new_pin_states = 0xFF
        self.GPIO_pub.publish(msg)
        
    def publish_pause(self, event):
        self.pause_pub.publish(std_msg.Bool(self.paused))
        
    def service_motion_mode_request(self, req):
        rospy.sleep(0.5)
        return req.mode
    
    def service_enable_wheelpods(self, req):
        rospy.sleep(0.5)
        return req.state
            
    def service_enable_carousel(self, req):
        rospy.sleep(0.5)
        return req.state

    def move_base(self, goal):
        target_pose = goal.target_pose
        fake_feedback = move_base_msg.MoveBaseFeedback()
        fake_feedback.base_position = target_pose
        rospy.sleep(5.0)
        self.move_base_server.publish_feedback(fake_feedback)
        self.move_base_server.set_succeeded()

    def run_visual_servo_action(self, goal):
        servo_result = VisualServoResult()
        servo_feedback = VisualServoFeedback()
    
        update_rate = rospy.Rate(10.0)
        fake_distance = 100
        step = 1
            
        while not rospy.is_shutdown():
            fake_distance = fake_distance - step
            if fake_distance <= 0:
                servo_result.success = True
                self.visual_servo_server.set_succeeded(result=servo_result)
                break
            else:
                servo_feedback.state = VisualServoStates.MOVE_FORWARD
                servo_feedback.error = fake_distance
                self.visual_servo_server.publish_feedback(servo_feedback)
                if self.visual_servo_server.is_preempt_requested():
                    break
            update_rate.sleep()
            
    def home_wheelpods(self, goal):
        fake_result = platform_msg.HomeResult([True,True,True])
        rospy.sleep(3.0)
        self.home_wheelpods_server.set_succeeded(result=fake_result)
        
    def home_carousel(self, goal):
        fake_result = platform_msg.HomeResult([True])
        rospy.sleep(3.0)
        self.home_carousel_server.set_succeeded(result=fake_result)
        
    def shutdown(self):
        rospy.signal_shutdown("Probably closed from terminal")
        
   
#dummy manipulator state, waits 10 seconds and
#exits with success unless preempted
class ManipulatorState(smach.State):
    def execute(self, userdata):
        start_time = rospy.get_time()
        #wait 2 seconds then return success
        while (rospy.get_time() - start_time) < 2:
            if self.preempt_requested():
                userdata.action_result = manipulator_msg.ManipulatorResult('fake_failure', False)
                return 'preempted'
            rospy.sleep(0.2)
        userdata.action_result = manipulator_msg.ManipulatorResult('fake_success', True)
        return 'succeeded'
    



       
