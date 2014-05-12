#!/usr/bin/env pyth
import sys
import math
import rospy
import rosnode
import actionlib
import tf
import tf_conversions
import numpy as np

import smach
import smach_ros

import samplereturn.util as util

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
        self.fake_sample = {'x':25, 'y':-5, 'name':'PRECACHED'}
        

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
        
        detected_sample_search_name = "/processes/sample_detection/detected_sample_search"
        detected_sample_manipulator_name = "/processes/sample_detection/detected_sample_manipulator"
        beacon_pose_name = "/processes/beacon_finder/beacon_pose"

        #tf stuff
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        
        self.zero_translation = (0,0,0)
        self.zero_rotation = (0,0,0,1)
                                                  
        self.fake_robot_pose = geometry_msg.Pose(geometry_msg.Point(0,0,0),
                                                 geometry_msg.Quaternion(0,0,0,1))
        
        rospy.Timer(rospy.Duration(0.1), self.broadcast_tf)        
       
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
        
        #sample and beacon detection stuff
        self.search_sample_pub = rospy.Publisher(detected_sample_search_name, samplereturn_msg.NamedPoint)
        rospy.Timer(rospy.Duration(1.0), self.publish_sample_detection_search)        

        self.manipulator_sample_pub = rospy.Publisher(detected_sample_manipulator_name, samplereturn_msg.NamedPoint)
        rospy.Timer(rospy.Duration(1.0), self.publish_sample_detection_manipulator)         
        
        beacon_rot = tf.transformations.quaternion_from_euler(0.0,
                                                              0.0,
                                                              0.0,
                                                             'rxyz')        
        self.fake_beacon_pose = geometry_msg.Pose(geometry_msg.Point(0,0,0),
                                                  geometry_msg.Quaternion(*beacon_rot))
        
        self.beacon_pose_pub = rospy.Publisher(beacon_pose_name, geometry_msg.PoseStamped)
        rospy.Timer(rospy.Duration(1.0), self.publish_beacon_pose)
                
                
        #visual servo stuff
        self.visual_servo_server = actionlib.SimpleActionServer(visual_servo_name,
                                                           visual_servo_msg.VisualServoAction,
                                                           self.run_visual_servo_action,
                                                           False)
        self.visual_servo_server.start()        

        #rospy.spin()
        
    def broadcast_tf(self, event):
        now = rospy.Time.now()
        
        self.tf_broadcaster.sendTransform(self.zero_translation,
                                          self.zero_rotation,
                                          now,
                                          '/odom',
                                          '/map')
        
        trans = (self.fake_robot_pose.position.x,
                       self.fake_robot_pose.position.y,
                       self.fake_robot_pose.position.z)
        
        rot = (self.fake_robot_pose.orientation.x,
                    self.fake_robot_pose.orientation.y,
                    self.fake_robot_pose.orientation.z,
                    self.fake_robot_pose.orientation.w)
        
        self.tf_broadcaster.sendTransform(trans, rot, now, '/base_link', '/odom')
        

        beacon_trans = (-1.3, 0, 1.0)
        beacon_rot = tf.transformations.quaternion_from_euler(0.0,
                                                              math.pi/2,
                                                              math.pi/2,
                                                              'rxyz')
        self.tf_broadcaster.sendTransform(beacon_trans,
                                          beacon_rot,
                                          now,
                                          '/beacon',
                                          '/map')

        self.tf_broadcaster.sendTransform((0,-1.0, 3.0),
                                          tf.transformations.quaternion_inverse(beacon_rot),
                                          now,
                                          '/platform',
                                          '/beacon')

    def publish_cam_status(self, event):
        if self.cameras_ready:
            msg = 'Ready'
        else:
            msg = 'Error'
        
        for pub in self.cam_publishers:
            pub.publish(std_msg.String(msg))        

    def publish_sample_detection_search(self, event):
        if self.publish_samples:
            msg = samplereturn_msg.NamedPoint()
            msg.header = std_msg.Header(0, rospy.Time.now(), '/map')
            msg.point = geometry_msg.Point(self.fake_sample['x'],
                                                   self.fake_sample['y'],
                                                   0.0)
            msg.name = self.fake_sample['name']
            self.search_sample_pub.publish(msg)
            
    def publish_sample_detection_manipulator(self, event):
        if self.publish_samples:
            msg = samplereturn_msg.NamedPoint()
            msg.header = std_msg.Header(0, rospy.Time.now(), '/map')
            msg.point = geometry_msg.Point(self.fake_sample['x'],
                                                   self.fake_sample['y'],
                                                   0.0)
            msg.name = self.fake_sample['name']
            self.manipulator_sample_pub.publish(msg)
            
    def publish_beacon_pose(self, event):
        msg = geometry_msg.PoseStamped()
        msg.header = std_msg.Header(0, rospy.Time.now(), '/beacon')
        msg.pose = self.fake_beacon_pose
        self.beacon_pose_pub.publish(msg)

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
        travel_time = 5.0        
        update_period = 0.1
        steps = int(travel_time / update_period)
 
        while True:
            target_pose = goal.target_pose
            start_pose = util.get_current_robot_pose(self.tf_listener)
            
            rospy.loginfo("SIMULATOR move base translation x,y:" + \
                          str(target_pose.pose.position.x - start_pose.pose.position.x) + ", " + \
                          str(target_pose.pose.position.y - start_pose.pose.position.y))
            q = target_pose.pose.orientation
            rpy = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
            rospy.loginfo("SIMULATOR move base target yaw:" + str(math.degrees(rpy[2])))
            x_list = np.linspace(start_pose.pose.position.x,
                                 target_pose.pose.position.x,
                                 num = steps)
            y_list = np.linspace(start_pose.pose.position.y,
                                 target_pose.pose.position.y,
                                 num = steps)
            for i in range(len(x_list)):
                rospy.sleep(update_period)
                if self.move_base_server.is_preempt_requested():
                    if self.move_base_server.is_new_goal_available():
                        rospy.loginfo("SIMULATOR new move_base goal received before complete")
                        goal = self.move_base_server.accept_new_goal()
                        break
                    else:
                        self.move_base_server.set_preempted()
                        return
                self.fake_robot_pose.position.x = x_list[i]
                self.fake_robot_pose.position.y = y_list[i]
                header = std_msg.Header(0, rospy.Time.now(), '/map')
                fake_feedback = move_base_msg.MoveBaseFeedback()
                fake_feedback.base_position.header = header
                fake_feedback.base_position.pose = self.fake_robot_pose
                self.move_base_server.publish_feedback(fake_feedback)
            
            if i >= (steps-1): break

        self.fake_robot_pose.orientation = target_pose.pose.orientation 
        self.move_base_server.set_succeeded()

    def run_visual_servo_action(self, goal):
        servo_result = visual_servo_msg.VisualServoResult()
        servo_feedback = visual_servo_msg.VisualServoFeedback()
    
        update_rate = rospy.Rate(10.0)
        fake_distance = 150
        step = 1
            
        while not rospy.is_shutdown():
            fake_distance = fake_distance - step
            if fake_distance <= 0:
                servo_result.success = True
                self.visual_servo_server.set_succeeded(result=servo_result)
                break
            else:
                servo_feedback.state = visual_servo_msg.VisualServoFeedback.MOVE_FORWARD
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
        
    def hide_samples(self):
        self.fake_sample['name'] = 'none'
        
    def show_samples(self):
        self.fake_sample['name'] = 'PREEMPTED'
        
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
    



       
