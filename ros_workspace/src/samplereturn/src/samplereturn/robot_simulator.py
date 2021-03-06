#!/usr/bin/env pyth
import sys
import math
import numpy as np
from numpy import trunc,pi,ones_like,zeros,linspace, eye, sin, cos, arctan2
from copy import deepcopy
import random
import collections

import smach
import smach_ros
import rospy
import rosnode
import actionlib
import tf
import tf2_ros
import tf_conversions
from cv_bridge import CvBridge, CvBridgeError

import samplereturn.util as util

import actionlib_msgs.msg as action_msg
import std_msgs.msg as std_msg
import sensor_msgs.msg as sensor_msg
import nav_msgs.msg as nav_msg
import sensor_msgs.msg as sensor_msg
import nav_msgs.srv as nav_srv
import platform_motion_msgs.msg as platform_msg
import platform_motion_msgs.srv as platform_srv
import manipulator_msgs.msg as manipulator_msg
import geometry_msgs.msg as geometry_msg
import move_base_msgs.msg as move_base_msg
import samplereturn_msgs.srv as samplereturn_srv
import visualization_msgs.msg as vis_msg

import sensor_msgs.point_cloud2 as pc2

import dynamic_reconfigure.srv as dynsrv
import dynamic_reconfigure.msg as dynmsg

from geometry_msgs.msg import Point, PointStamped, TransformStamped, Transform
from samplereturn_msgs.msg import NamedPoint, PursuitResult

class RobotSimulator(object):
    
    def __init__(self, mode='level_two', publish_samples=True, publish_beacon=True):
    
        rospy.init_node("robot_simulator")

        #IMPORTANT SWITCHES!        
        self.publish_samples = publish_samples
        self.random_sample_verify = False
        self.publish_beacon = publish_beacon
        self.odometry_is_noisy = True
        self.broadcast_localization = False #fake localization handled by beacon_localizer!
        self.reality_frame = 'map'
        self.sim_map = 'sim_map'
        self.sim_odom = 'odom'
        
        #simulator state variables and constants            
        self.GPIO_LEVEL_ONE = 0x20
        self.GPIO_LEVEL_TWO = 0x08

        #status flags
        self.mode = mode
        self.cameras_ready = True
        self.paused = False
        
        self.active_sample_id = None
        self.collected_ids = []
        self.excluded_ids = []
        self.fake_samples = [{'point':geometry_msg.Point(-27, 58, 0),'id':16},
                             {'point':geometry_msg.Point(-105, 62, 0),'id':1},
                             {'point':geometry_msg.Point(-105, 34, 0), 'id':2},
                             {'point':geometry_msg.Point(-84, 67, 0), 'id':3},
                             {'point':geometry_msg.Point(-51, 64, 0), 'id':4},
                             {'point':geometry_msg.Point(-29, 31, 0), 'id':5},
                             {'point':geometry_msg.Point(-29, -11, 0), 'id':6},
                             {'point':geometry_msg.Point(14, 37, 0), 'id':7},
                             {'point':geometry_msg.Point(39, 3, 0), 'id':8},
                             {'point':geometry_msg.Point(29, -24, 0), 'id':9},
                             {'point':geometry_msg.Point(65, -34, 0), 'id':10},
                             {'point':geometry_msg.Point(73, -60, 0), 'id':11},
                             {'point':geometry_msg.Point(104, -25, 0), 'id':12},
                             {'point':geometry_msg.Point(133, 9, 0), 'id':13},
                             {'point':geometry_msg.Point(138, 31, 0), 'id':14},
                             {'point':geometry_msg.Point(164, 42, 0), 'id':15}]

        self.sample_marker = vis_msg.Marker()
        self.sample_marker.header = std_msg.Header(0, rospy.Time(0), self.reality_frame)
        self.sample_marker.type = vis_msg.Marker.CYLINDER
        self.sample_marker.scale = geometry_msg.Vector3(.6, .6, .05)
        self.sample_marker.pose.orientation = geometry_msg.Quaternion(0,0,0,1)
        self.sample_marker.lifetime = rospy.Duration(1.5)
         
        self.path_marker = vis_msg.Marker()
        self.path_marker.header = std_msg.Header(0, rospy.Time(0), self.reality_frame)
        self.path_marker.type = vis_msg.Marker.ARROW
        self.path_marker.color = std_msg.ColorRGBA(0, 0, 1, 1)
        self.path_marker.scale = geometry_msg.Vector3(1.0, .2, .2)
        self.path_marker.lifetime = rospy.Duration(0)
                                                        
        self.joint_state_seq = 0
        
        self.motion_mode = platform_srv.SelectMotionModeRequest.MODE_ENABLE

        #topic and action names
        manipulator_action_name = "/motion/manipulator/grab_action"
        home_wheelpods_name = "/motion/wheel_pods/home"
        home_carousel_name = "/motion/carousel/home"
        enable_wheelpods_name = "/motion/wheel_pods/enable"
        enable_carousel_name = "/motion/carousel/enable"

        enable_manipulator_detector_name = "/processes/sample_detection/manipulator/saliency_detector/enable"
        enable_manipulator_projector_name = "/processes/sample_detection/manipulator/pointcloud_projector/enable"              
        
        enable_search_name = "/cameras/search/enable_publish"
        enable_navigation_beacon_name =  "/cameras/navigation/beacon/basler_camera/enable_publish"
        
        select_motion_name = "/motion/CAN/select_motion_mode"
        
        joint_state_name = "/motion/platform_joint_state"
        odometry_name = '/motion/odometry'

        planner_command_name = '/motion/planner_command'
        servo_command_name = '/motion/servo_command'
        joystick_command_name = '/motion/joystick_command'

        cam_status_list = ["/cameras/navigation/port/status",
                                "/cameras/navigation/center/status",
                                "/cameras/navigation/starboard/status",
                                "/cameras/manipulator/status",
                                "/cameras/search/status"]
        
        gpio_read_name = "/io/gpio_read"
        pause_state_name = "/io/pause_state"
        
        pursuit_result_name = "/processes/executive/pursuit_result"
        detected_sample_search_name = "/processes/sample_detection/search/filtered_point"
        detected_sample_manipulator_name = "/processes/sample_detection/manipulator/filtered_point"
        search_verify_name = "/processes/sample_detection/search/close_range_verify"
        beacon_pose_name = "/processes/beacon/beacon_pose"
        beacon_debug_pose_name = "/processes/beacon/beacon_pose_debug"

        manipulator_image_name = "/cameras/manipulator/left/rect_color"
       
        point_cloud_center_name = "/cameras/navigation/center/points2"
        point_cloud_port_name = "/cameras/navigation/port/points2"
        point_cloud_starboard_name = "/cameras/navigation/starboard/points2"
        
        time_remaining_name = '/processes/executive/minutes_remaining'
        self.time_remaining = 0

        #tf stuff
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        
        self.zero_translation = geometry_msg.Vector3(0,0,0)
        self.zero_rotation = geometry_msg.Quaternion(0,0,0,1)

        self.test_translation = geometry_msg.Vector3(4.0,4.0,0)
        self.test_rotation = geometry_msg.Quaternion(
                             *tf.transformations.quaternion_from_euler(0,0,0.1))

        #odometry
        self.odometry_dt = 0.05
        odometry_noise_sigma = np.diag([0.1, 0.1, 0.01, 0.01])
        self.odometry_noise_mean = [0.0, 0.0, 0.0002, 0.0]
        self.odometry_noise_covariance = np.square(odometry_noise_sigma*self.odometry_dt)
        self.robot_pose = self.initial_pose()
        self.robot_odometry = self.initial_odometry()
        self.noisy_robot_pose = self.initial_pose()
        self.noisy_robot_odometry = self.initial_odometry()
        self.command_twist = geometry_msg.Twist()
       
        #rospy.logwarn("ODOM_NOISE_COVAR: {!s}".format(self.odometry_noise_covariance))

        #manipulator stuff
        self.manipulator_sm = smach.StateMachine(
            outcomes=['success', 'aborted', 'preempted'],
            input_keys = ['action_goal'],
            output_keys = ['action_result'])
        
        with self.manipulator_sm:

            smach.StateMachine.add('START_MANIPULATOR',
                                    ManipulatorState(self.set_sample_success),
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
       
        #camera publishers, status and detections are sent in here.
        #detections are added to a list in their respective check callbacks
        self.search_sample_queue = collections.deque()
        self.search_sample_delay = rospy.Duration(3.0)
        self.beacon_pose_queue = collections.deque()
        self.beacon_pose_delay = rospy.Duration(1.0)
        self.cam_publishers = []
        for topic in cam_status_list:
            self.cam_publishers.append(rospy.Publisher(topic, std_msg.String, queue_size=2))
        rospy.Timer(rospy.Duration(0.1), self.publish_camera_messages)
        
        rospy.Subscriber(enable_search_name,
                         std_msg.Bool,
                         self.handle_search_enable)

        rospy.Subscriber(enable_navigation_beacon_name,
                         std_msg.Bool,
                         self.handle_beacon_enable)        
        self.beacon_enabled = True
        self.search_enabled = False
        
        rospy.Service(search_verify_name,
                      samplereturn_srv.Verify,
                      self.service_search_verify_request)
        rospy.Service(enable_manipulator_detector_name,
                      samplereturn_srv.Enable,
                      self.enable_manipulator_detector)
        rospy.Service(enable_manipulator_projector_name,
                      samplereturn_srv.Enable,
                      self.enable_manipulator_projector)
        self.manipulator_detector_enabled = False
        
        #publisher for blank images to sun_pointing
        self.cv_bridge = CvBridge()
        self.manipulator_image_publisher = rospy.Publisher(manipulator_image_name,
                                                           sensor_msg.Image,
                                                           queue_size = 1)
        self.empty_image_data = np.zeros((480,640,3), dtype='uint8')
        
        rospy.Timer(rospy.Duration(1.0/30.0), self.publish_manipulator_image)
                                            
        #io publishers
        self.GPIO_pub = rospy.Publisher(gpio_read_name, platform_msg.GPIO, queue_size=2)
        rospy.Timer(rospy.Duration(0.2), self.publish_GPIO)
        self.pause_pub = rospy.Publisher(pause_state_name, std_msg.Bool, latch=True, queue_size=2)
        
        #platform motion publishers, services, and action servers
        rospy.Service(select_motion_name,
                      platform_srv.SelectMotionMode,
                      self.service_motion_mode_request)
        rospy.Service(enable_wheelpods_name,
                      platform_srv.Enable,
                      self.service_enable_wheelpods)
        rospy.Service(enable_carousel_name,
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
        
        self.joint_state_pub = rospy.Publisher(joint_state_name,
                                               sensor_msg.JointState,
                                               queue_size=2)
        
        #planner, odom and tf
        self.planner_sub = rospy.Subscriber(planner_command_name,
                                            geometry_msg.Twist,
                                            self.handle_planner)
        
        self.servo_sub = rospy.Subscriber(servo_command_name,
                                            geometry_msg.Twist,
                                            self.handle_servo)
        
        self.joystick_sub = rospy.Subscriber(joystick_command_name,
                                            geometry_msg.Twist,
                                            self.handle_joystick)

        self.odometry_pub = rospy.Publisher(odometry_name,
                                            nav_msg.Odometry,
                                            queue_size=2)

        self.joint_transforms_available = False
        rospy.Timer(rospy.Duration(self.odometry_dt), self.broadcast_tf_and_motion)

        #sample detection publishers
        self.search_sample_pub = rospy.Publisher(detected_sample_search_name,
                                                 NamedPoint,
                                                 queue_size=10)
        rospy.Timer(rospy.Duration(1.0), self.check_sample_detection_search)        

        self.manipulator_sample_pub = rospy.Publisher(detected_sample_manipulator_name,
                                                      NamedPoint,
                                                      queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.publish_sample_detection_manipulator)         
        
        self.sample_marker_pub = rospy.Publisher('fake_samples',
                                                 vis_msg.Marker,
                                                 queue_size=30)
        
        self.pursuit_result_sub = rospy.Subscriber(pursuit_result_name,
                                                   PursuitResult,
                                                   self.handle_pursuit_result)
        
        self.time_remaining_sub = rospy.Subscriber(time_remaining_name,
                                                   std_msg.Int16,
                                                   self.handle_time_remaining)
       
        self.points_center_pub = rospy.Publisher(point_cloud_center_name,
                                                 sensor_msg.PointCloud2,
                                                 queue_size=2)
        self.points_port_pub = rospy.Publisher(point_cloud_port_name,
                                                 sensor_msg.PointCloud2,
                                                 queue_size=2)
        self.points_starboard_pub = rospy.Publisher(point_cloud_starboard_name,
                                                 sensor_msg.PointCloud2,
                                                 queue_size=2)
        
        #map stuff
        print "Waiting for map server"
        rospy.wait_for_service('/processes/planner/static_map')
        self.get_map = rospy.ServiceProxy('/processes/planner/static_map', nav_srv.GetMap)
        self.global_map = self.get_map().map
        print("map info: " + str(self.global_map.info))

        #wait for robot_state publisher
        while not rospy.is_shutdown():
            try:
                self.tf_listener.waitForTransform('base_link',
                                                  "port_suspension",
                                                  rospy.Time(0),
                                                  rospy.Duration(1.0))
                rospy.logdebug("Waiting for robot_state_publisher transforms")
                break
            except (tf.Exception):
                rospy.logdebug("No robot_state_publisher transforms")
                pass
        
        (x, y,_),_ =  self.tf_listener.lookupTransform('base_link', 'port_suspension', rospy.Time(0))
        self.port_vector = np.array([x,y,0])
        (x, y,_),_ =  self.tf_listener.lookupTransform('base_link', 'starboard_suspension', rospy.Time(0))
        self.starboard_vector = np.array([x,y,0])
        (x, y,_),_ =  self.tf_listener.lookupTransform('base_link', 'stern_suspension', rospy.Time(0))
        self.stern_vector = np.array([x,y,0])

        self.joint_transforms_available = True
        
        self.path_counter = 0
        self.path_marker_pub = rospy.Publisher('path_markers',
                                               vis_msg.Marker,
                                               queue_size=10)
        rospy.Timer(rospy.Duration(3.0), self.publish_path_markers)

        rospy.Timer(rospy.Duration(0.15), self.publish_point_cloud)        
        
        #beacon publishers
        self.beacon_pose_pub = rospy.Publisher(beacon_pose_name,
                                               geometry_msg.PoseWithCovarianceStamped,
                                               queue_size=2)
        self.beacon_debug_pose_pub = rospy.Publisher(beacon_debug_pose_name,
                                                     geometry_msg.PoseStamped,
                                                     queue_size=2)
        rospy.Timer(rospy.Duration(1.0), self.check_beacon_pose)
        
        #this should latch and start the sim unpaused
        self.unpause()
             
        #rospy.spin()
       
    def publish_path_markers(self, event):
        try:
            self.path_marker.pose = util.get_current_robot_pose(self.tf_listener, self.reality_frame).pose
        except:
            return
        self.path_marker.id = self.path_counter
        self.path_counter += 1
        self.path_marker_pub.publish(self.path_marker)
 
    def handle_planner(self, twist):
        if self.motion_mode == platform_srv.SelectMotionModeRequest.MODE_PLANNER_TWIST:
            self.command_twist = twist
        
    def handle_servo(self, twist):
        if self.motion_mode == platform_srv.SelectMotionModeRequest.MODE_SERVO:
            self.command_twist = twist

    def handle_joystick(self, twist):
        if self.motion_mode == platform_srv.SelectMotionModeRequest.MODE_JOYSTICK:
            self.command_twist = twist

    def publish_point_cloud(self, event):
        #rospy.loginfo("Starting PC generation")

        now = event.current_real
        #get latest available transforms to map
        header = std_msg.Header(0, rospy.Time(0), self.reality_frame)
        target_frame = 'navigation_center_left_camera'
        try:
            current_pose = util.get_current_robot_pose(self.tf_listener,
                    self.reality_frame)
            transform = self.tf_listener.asMatrix(target_frame, header)
        except ( tf.Exception ), e:
            #rospy.loginfo("Failed to transform map for PC")
            #rospy.loginfo("Exception: %s"%e)
            return
        header.stamp = now #set broadcasted transform time to now
        center_cloud = self.get_pointcloud2(self.global_map,
                                            current_pose.pose.position,
                                            target_frame,
                                            transform)
        self.points_center_pub.publish(center_cloud)
        header = std_msg.Header(0, now, 'navigation_port_left_camera')
        empty_cloud = pc2.create_cloud_xyz32(header,[])
        self.points_port_pub.publish(empty_cloud)
        empty_cloud.header.frame_id = 'navigation_starboard_left_camera'
        self.points_starboard_pub.publish(empty_cloud)
        
        #interval = rospy.get_time() - now.to_sec()
        #rospy.loginfo("Finished PC generation in: %f"%interval)
               
    def broadcast_tf_and_motion(self, event):
        #rospy.loginfo("Starting TF and Odom")
        now = rospy.Time.now()
        transforms = []
        if self.broadcast_localization:
            transform = TransformStamped(std_msg.Header(0, now, self.sim_map),
                                         self.sim_odom,
                                         Transform(self.test_translation,
                                                   self.test_rotation))
            transforms.append(transform)

        self.integrate_odometry()
        
        transform = TransformStamped(std_msg.Header(0, now, self.sim_odom),
                                     'base_link',
                                     Transform(self.robot_pose.pose.position,
                                               self.robot_pose.pose.orientation))
        transforms.append(transform)

        #broadcast the clean base_link in map for comparison
        transform = TransformStamped(std_msg.Header(0, now, self.reality_frame),
                                     'perfect_odometry_in_static_frame',
                                     Transform(self.robot_pose.pose.position,
                                               self.robot_pose.pose.orientation))
        transforms.append(transform)

        if self.odometry_is_noisy:
 
            qn = (self.noisy_robot_pose.pose.orientation.x,
                  self.noisy_robot_pose.pose.orientation.y,
                  self.noisy_robot_pose.pose.orientation.z,
                  self.noisy_robot_pose.pose.orientation.w)
            tn = (self.noisy_robot_pose.pose.position.x,
                  self.noisy_robot_pose.pose.position.y,
                  self.noisy_robot_pose.pose.position.z)
            noise_to_odom = tf.transformations.compose_matrix(
                    angles=tf.transformations.euler_from_quaternion(qn),
                    translate=tn)

            qp = (self.robot_pose.pose.orientation.x,
                  self.robot_pose.pose.orientation.y,
                  self.robot_pose.pose.orientation.z,
                  self.robot_pose.pose.orientation.w)
            tp = (self.robot_pose.pose.position.x,
                  self.robot_pose.pose.position.y,
                  self.robot_pose.pose.position.z)
            base_to_odom = tf.transformations.compose_matrix(
                            angles=tf.transformations.euler_from_quaternion(qp),
                            translate=tp)
            
            #a transform to show where the integrated noise takes us in real map
            _,_,noise_angles, noise_translate,_ = tf.transformations.decompose_matrix(noise_to_odom)
            noise_rot = tf.transformations.quaternion_from_euler(*noise_angles)

            transform = TransformStamped(std_msg.Header(0, now, self.sim_map),
                                         'noisy_odometry_in_sim_map',
                                         Transform(geometry_msg.Vector3(*noise_translate),
                                                   geometry_msg.Quaternion(*noise_rot)))
            transforms.append(transform)

            #calculate the transform between clean and noisy base_link, in odom
            noise_transform = np.dot(noise_to_odom, np.linalg.inv(base_to_odom))
            
            #publish the transform for visualization
            _, _, nt_angles, nt_translate, _ = tf.transformations.decompose_matrix(noise_transform)
            nt_rot = tf.transformations.quaternion_from_euler(*nt_angles)
            transform = TransformStamped(std_msg.Header(0, now, self.reality_frame),
                                         'noise_transform',
                                         Transform(geometry_msg.Vector3(*nt_translate),
                                                   geometry_msg.Quaternion(*nt_rot)))
            transforms.append(transform)

            #get the current correction broadcast by the localizer (map->odom)
            try:
                pos, quat = self.tf_listener.lookupTransform(self.sim_odom, self.sim_map, rospy.Time(0))
                #get the matrix describing map->odom
                map_to_odom = tf.transformations.compose_matrix(
                              angles=tf.transformations.euler_from_quaternion(quat),
                              translate=pos)

                #apply the correction map->odom
                error = np.dot(noise_transform, map_to_odom)

                #this is the map->sim_map transform, which should be the error between the
                #integrated noise and the localization corrections
                _, _, error_angles, error_translate, _ = tf.transformations.decompose_matrix(error)
                error_rot = tf.transformations.quaternion_from_euler(*error_angles)
    
                transform = TransformStamped(std_msg.Header(0, now, self.reality_frame),
                                             self.sim_map,
                                             Transform(geometry_msg.Vector3(*error_translate),
                                                       geometry_msg.Quaternion(*error_rot)))
                transforms.append(transform)

            except tf.Exception, exc:
                #if the transform isn't availabe, no correction applied
                rospy.logdebug("NO map->odom transform: {!s}".format(exc))            
                            
        else:
            transform = TransformStamped(std_msg.Header(0, now, self.reality_frame),
                                         self.sim_map,
                                         Transform(self.zero_translation,
                                                   self.zero_rotation))
            transforms.append(transform)
  
        #broadcast transformlist
        self.tf_broadcaster.sendTransform(transforms)
  
        #also publish odometry
        self.odometry_pub.publish(self.robot_odometry)

        #publish joint states
        self.publish_joint_states(self.robot_odometry)
        
        #interval = rospy.get_time() - now.to_sec()
        #rospy.loginfo("Finished TF and Odom in: %f"%interval)

    def publish_camera_messages(self, event):
        if self.cameras_ready:
            msg = 'Ready'
        else:
            msg = 'Error'
        
        for pub in self.cam_publishers:
            pub.publish(std_msg.String(msg))
        
        now = rospy.Time.now()
        
        #see if it's time to publish a sample detection    
        if (len(self.search_sample_queue) > 0):
            delay = now - self.search_sample_queue[0].header.stamp
            if (delay > self.search_sample_delay):
                #rospy.loginfo("Publishing Sample: {!s} with delay {!s}".format(self.search_sample_queue[0].header, delay.to_sec()))
                self.search_sample_pub.publish(self.search_sample_queue.popleft())
        
        #see if it's time to publish a beacon detection
        if (len(self.beacon_pose_queue) > 0):
            delay = now - self.beacon_pose_queue[0].header.stamp
            if ((now - self.beacon_pose_queue[0].header.stamp) > self.beacon_pose_delay):
                #rospy.loginfo("Publishing Beacon: {!s} with delay {!s}".format(self.beacon_pose_queue[0].header, delay.to_sec()))
                self.beacon_pose_pub.publish(self.beacon_pose_queue.popleft())
    
    def publish_manipulator_image(self, event):
        now = event.current_real
        header = std_msg.Header(0, now, 'manipulator_left_camera')
        msg = self.cv_bridge.cv2_to_imgmsg(self.empty_image_data, "bgr8")
        msg.header = header
        self.manipulator_image_publisher.publish(msg)

    def check_sample_detection_search(self, event):
        if self.publish_samples and self.search_enabled:
            for sample in self.fake_samples:
                #get headers in Time(0) for latest transforms
                header = std_msg.Header(0, rospy.Time(0), self.reality_frame)    
                self.sample_marker.header = header
                self.sample_marker.pose.position = sample['point']
                self.sample_marker.id = sample['id']
                self.sample_marker.text = 'sample: ' + str(sample['id'])                
                self.sample_marker.color = std_msg.ColorRGBA(0, 0, 254, 1)
                
                if sample['id'] in self.collected_ids:
                    self.sample_marker.color = std_msg.ColorRGBA(0, 254, 0, 1)
                elif sample['id'] in self.excluded_ids:   
                    self.sample_marker.color = std_msg.ColorRGBA(254, 0, 0, 1)
                else:
                    if (self.sample_in_view(sample['point'], 1, 2.5, 3)) \
                    or (sample['id'] == self.active_sample_id):
                        #keep publishing the active detection id until it is cleared
                        #hopefully the sim won't have more than one active id...
                        self.sample_marker.color = std_msg.ColorRGBA(254, 0, 254, 1)
                        sample_in_map = PointStamped(header, sample['point'])
                        try:
                            sample_point_odom = self.tf_listener.transformPoint(self.sim_odom, sample_in_map)
                        except tf.Exception:
                            rospy.logwarn("SIMULATOR failed to transform search detection point")
                            break
                            
                        #append the detection to the delayed queue, with a now timestamp
                        header.stamp = rospy.Time.now()
                        msg = NamedPoint(header = sample_point_odom.header,
                                         point = sample_point_odom.point,
                                         filter_id = sample['id'] + 100,
                                         sample_id = sample['id'])
                        self.active_sample_id = sample['id']
                        self.search_sample_queue.append(msg)
                                        
                self.sample_marker_pub.publish(self.sample_marker)
                
    def publish_sample_detection_manipulator(self, event):
        if self.publish_samples and self.manipulator_detector_enabled:
            for sample in self.fake_samples:
                if not sample['id'] in self.collected_ids:
                     if self.sample_in_view(sample['point'], -0.1, 0.5, 0.2):
                        header = std_msg.Header(0, rospy.Time(0), self.reality_frame)
                        sample_in_map = PointStamped(header, sample['point'])
                        try:
                            sample_point_odom = self.tf_listener.transformPoint(self.sim_odom, sample_in_map)
                        except tf.Exception:
                            rospy.logwarn("SIMULATOR failed to transform manipulator detection point")
                        header.stamp = rospy.Time.now()
                        msg = NamedPoint(header = sample_point_odom.header,
                                         point = sample_point_odom.point,
                                         sample_id = sample['id'])
                        self.manipulator_sample_pub.publish(msg)
            
    def sample_in_view(self, point, min_x, max_x, max_y):
        header = std_msg.Header(0, rospy.Time(0), self.reality_frame)
        point_stamped = geometry_msg.PointStamped(header, point)
        try:
            base_relative = self.tf_listener.transformPoint('base_link',
                                                            point_stamped)
        except:
            return False
        x = base_relative.point.x
        y = base_relative.point.y
        return ( ((x > min_x) and (x < max_x)) and (np.abs(base_relative.point.y) < max_y) )        

    def handle_pursuit_result(self, msg):
        if msg.success:
            #collected samples are added by the manipulator grab action
            print "Received success message for sample: " + str(self.active_sample_id)
            print "Collected IDs: %s" % (self.collected_ids)
        else:
            if self.active_sample_id is not None:
                self.excluded_ids.append(self.active_sample_id)
            self.search_sample_queue.clear()
            print "Received failure message for sample: " + str(self.active_sample_id)
            print "Excluded IDs: %s" % (self.excluded_ids) 
        self.active_sample_id = None
        print ("Time remaining: {!s} minutes".format(self.time_remaining))
        
    def handle_time_remaining(self, msg):
        self.time_remaining = msg.data
        
    def check_beacon_pose(self, event):
        if not self.publish_beacon:
            return

        if not self.beacon_enabled:
            return
                
        try:
            #get distances and yaws from reality frame origin
            platform_origin = PointStamped(std_msg.Header(0, rospy.Time(0),
                                                          'platform'),
                                                          Point(0,0,0))
            angle_to_platform, dist_from_platform = util.get_robot_strafe(self.tf_listener,
                                                                          platform_origin)
            angle_to_robot = util.get_robot_yaw_from_origin(self.tf_listener,
                                                            'platform')
        except tf.Exception, exc:
                print("Transforms not available in publish beacon: {!s}".format(exc))
                return

        '''
        rospy.loginfo("BEACON CHECK, dist_to_origin, angle_to_robot, angle_to_origin:\
                      {!s}, {!s}, {!s}".format(dist_from_origin,
                                        np.degrees(angle_to_robot),
                                        np.degrees(angle_to_origin)))
        '''

        if dist_from_platform < 5.0 or dist_from_platform > 50.0:
            #print ("NO BEACON PUB: outside beacon view distance: %.2f" %(dist_from_origin))
            return
        
        # within camera FOV
        if angle_to_platform > np.radians(-20.0) and angle_to_platform < np.radians(20.0):

            #get beacon covar and pose from beacon_finder launch
            #then create the message covariance
            position_sigma = rospy.get_param("/processes/beacon/april_beacon_finder/position_sigma")
            position_sigma_scale = rospy.get_param("/processes/beacon/april_beacon_finder/position_sigma_scale")
            rotation_sigma = rospy.get_param("/processes/beacon/april_beacon_finder/rotation_sigma_3tag")

            pos_covariance = (position_sigma + dist_from_platform*position_sigma_scale)**2
            rot_covariance = rotation_sigma**2
            
            diagonal = [pos_covariance]*3 + [rot_covariance]*3
            cov = np.diag(diagonal)
            msg_cov = list(cov.flat)
            
            #get the next search_camera transform, and call that the 'picture time'
            #then load the beacon pose (in true map), and transform to search_camera_lens
            try:
                beacon_trans, beacon_quat = \
                            self.tf_listener.lookupTransform(self.sim_map,
                                                             'beacon',
                                                             rospy.Time(0))
                now = self.tf_listener.getLatestCommonTime('navigation_beacon_lens',
                                                            self.reality_frame,)
                msg = geometry_msg.PoseWithCovarianceStamped()
                msg_pose = geometry_msg.PoseStamped()
                msg_pose.header = std_msg.Header(0, now, self.reality_frame)
                msg_pose.pose.orientation = geometry_msg.Quaternion(*beacon_quat)
                msg_pose.pose.position = geometry_msg.Point(*beacon_trans)
                msg_pose = self.tf_listener.transformPose('navigation_beacon_lens', msg_pose)
            except tf.Exception, exc:
                print("Transforms not available in publish beacon: {!s}".format(exc))
                return
            msg.pose.pose = msg_pose.pose
            msg.header = msg_pose.header
            msg.pose.covariance = msg_cov
            #append the detection to the delayed queue
            self.beacon_pose_queue.append(msg)
            self.beacon_debug_pose_pub.publish(msg_pose)
        #else:
            #print ("NO BEACON PUB: not in search view, angle_to_origin: %.2f" %(angle_to_origin))

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
        
    def pause(self):
        self.pause_pub.publish(std_msg.Bool(True))
        
    def unpause(self):
        self.pause_pub.publish(std_msg.Bool(False))
 
    def enable_manipulator_detector(self, req):
        self.manipulator_detector_enabled = req.state
        rospy.sleep(0.25)
        return req.state

    def enable_manipulator_projector(self, req):
        rospy.sleep(0.05)
        return req.state

    #camera enable handlers
    def handle_beacon_enable(self, msg):
        self.beacon_enabled = msg.data

    def handle_search_enable(self, msg):
        self.search_enabled = msg.data
            
    #fail 1 out of 3 times to verify a sample
    def service_search_verify_request(self, req):
        rospy.sleep(0.5)
        if self.random_sample_verify:
            return random.choice([True, True, False])
        else:
            return True
        
    def service_motion_mode_request(self, req):
        rospy.sleep(0.2)
        #print "set mode %d %d %d"%(platform_srv.SelectMotionModeRequest.MODE_PLANNER_TWIST,
        #        self.motion_mode, req.mode)
        if req.mode == platform_srv.SelectMotionModeRequest.MODE_QUERY:
            return self.motion_mode
        else:
            save_modes = [platform_srv.SelectMotionModeRequest.MODE_HOME,
                          platform_srv.SelectMotionModeRequest.MODE_JOYSTICK,
                          platform_srv.SelectMotionModeRequest.MODE_PLANNER_TWIST,
                          platform_srv.SelectMotionModeRequest.MODE_SERVO,
                          platform_srv.SelectMotionModeRequest.MODE_ENABLE]
            if req.mode in save_modes:
                self.saved_motion_mode = req.mode
            if req.mode == platform_srv.SelectMotionModeRequest.MODE_RESUME:
                self.motion_mode = self.saved_motion_mode
            else:
                self.motion_mode = req.mode
            return self.motion_mode
    
    def service_enable_wheelpods(self, req):
        rospy.sleep(0.5)
        return req.state
            
    def service_enable_carousel(self, req):
        rospy.sleep(0.5)
        return req.state
 
    def set_sample_success(self):
        if self.active_sample_id is not None:
            self.collected_ids.append(self.active_sample_id)
            self.search_sample_queue.clear()
            rospy.sleep(1.0)
            #ensure that the publisher is finished
     
    #homing request for wheelpods happens at beginning, zero useful sim values        
    def home_wheelpods(self, goal):
        self.zero_robot()
        self.collected_ids = []
        self.excluded_ids = []
        self.active_sample_id = None
        fake_result = platform_msg.HomeResult([True,True,True])
        rospy.sleep(1.0)
        self.home_wheelpods_server.set_succeeded(result=fake_result)
        
    def home_carousel(self, goal):
        fake_result = platform_msg.HomeResult([True])
        rospy.sleep(1.0)
        self.home_carousel_server.set_succeeded(result=fake_result)
        
    def hide_samples(self):
        self.publish_samples = False
        
    def show_samples(self):
        self.publish_samples = True
        
    def hide_beacon(self):
        self.publish_beacon = False
        
    def show_beacon(self):
        self.publish_beacon = True
            
    def shift_samples(self, x=0.3, y=0.3):
        for sample in self.fake_samples:
            sample['point'].x += x
            sample['point'].y += y
            
    def set_sample_delay(self, delay):
        self.search_sample_delay = rospy.Duration(delay)
        
    def shutdown(self):
        rospy.signal_shutdown("Probably closed from terminal")
        
    def zero_robot(self):
        self.robot_pose = self.initial_pose()
        self.robot_odometry = self.initial_odometry()
        self.command_twist = geometry_msg.Twist()
        self.noisy_robot_pose = self.initial_pose()
        self.noisy_robot_odometry = self.initial_odometry()
        

    def get_pointcloud2(self, grid, position, target_frame, transform, range=12.0):
        header =  std_msg.Header(0, rospy.Time.now(), target_frame)
        
        grid_np = np.array(grid.data, dtype='u1').reshape((grid.info.height,grid.info.width))
        origin = np.array((grid.info.origin.position.x,
                           grid.info.origin.position.y,
                           grid.info.origin.position.z))
        world2map = lambda x:np.clip(trunc((x-origin)/grid.info.resolution),
                           zeros((3,)),
                           np.r_[grid.info.width-1, grid.info.height-1,0])       
        
        ll = world2map(np.r_[position.x-range, position.y-range, 0])
        ur = world2map(np.r_[position.x+range, position.y+range, 0])
    
        submap = grid_np[ll[1]:ur[1],ll[0]:ur[0]]
        mappts = np.c_[np.where(submap==100)][:,::-1] + ll[:2]
        map2world = lambda x:(x+0.5)*grid.info.resolution+origin[:2]
        wpts = np.array([map2world(x) for x in mappts])
        pcpts=[]
        if len(wpts>0):
            for z in np.linspace(0,1,11):
                pcpts.append(np.c_[wpts,z*ones_like(wpts[:,0])])
            pcpts = np.vstack(pcpts)
            pcpts = np.einsum("ki,...ji", transform, np.c_[pcpts,np.ones((len(pcpts),1))])[:,:3]   
        else:
            pcpts = []
        pc = pc2.create_cloud_xyz32(header, pcpts)
        return pc

    def initial_odometry(self):
        odom = nav_msg.Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = self.sim_odom
        odom.child_frame_id = 'base_link'
        odom.pose.pose.orientation.w=1.0
        odom.pose.covariance = list(eye(6).reshape((36,)))
        odom.twist.covariance = list(eye(6).reshape((36,)))
        return odom
    
    def initial_pose(self):
        pose = geometry_msg.PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.sim_odom
        pose.pose.orientation.w = 1.0
        return pose
    
    def integrate_odometry(self):
        now = rospy.Time.now()
        itwist = self.command_twist
        self.robot_pose, self.robot_odometry = \
                self.integrate_twist(self.robot_pose,
                        self.robot_odometry,
                        itwist, now)
        self.noisy_robot_pose, self.noisy_robot_odometry = \
                self.integrate_twist(self.noisy_robot_pose,
                        self.noisy_robot_odometry,
                        self.odometry_noise(itwist), now)

    def odometry_noise(self, twist):
        if( np.hypot(twist.linear.x, twist.linear.y) > 1e-2 ):
            strafe_dir = np.arctan2(twist.linear.y,
                                    twist.linear.x)
            noise = np.random.multivariate_normal(self.odometry_noise_mean,
                    self.odometry_noise_covariance)
            ss = np.sin(strafe_dir)
            cs = np.cos(strafe_dir)
            noise = np.r_[noise[0]*cs-noise[1]*ss,
                          noise[0]*ss+noise[1]*cs,
                          noise[2]]
        elif np.abs(twist.angular.z)>1e-2:
            noise = np.random.multivariate_normal(self.odometry_noise_mean,
                    self.odometry_noise_covariance)
            noise[2] = noise[3]
        else:
            noise = np.zeros((3,))
        twist.linear.x += noise[0]
        twist.linear.y += noise[1]
        twist.angular.z += noise[2]
        return twist


    def integrate_twist(self, pose, odometry, twist, now):
        #print twist.linear.x, twist.linear.y, twist.angular.z
        dt = (now - pose.header.stamp).to_sec()
        yaw = 2*arctan2(pose.pose.orientation.z,
                        pose.pose.orientation.w)
        cy = cos(yaw)
        sy = sin(yaw)
        pose.pose.position.x += (cy*odometry.twist.twist.linear.x*dt
                                -sy*odometry.twist.twist.linear.y*dt)
        pose.pose.position.y += (sy*odometry.twist.twist.linear.x*dt
                                +cy*odometry.twist.twist.linear.y*dt)
        yaw += odometry.twist.twist.angular.z*dt
        pose.pose.orientation.w = cos(yaw/2)
        pose.pose.orientation.z = sin(yaw/2)
        pose.header.stamp = now
        odometry.pose.pose = deepcopy(pose.pose)
        odometry.twist.twist = deepcopy(twist)
        odometry.header.stamp = now
        return pose, odometry


    def publish_joint_states(self, current_odometry):

        self.joint_state_seq += 1
        joints = sensor_msg.JointState()
        header = std_msg.Header(self.joint_state_seq,
                                current_odometry.header.stamp,
                                'base_link')

        if self.joint_transforms_available:
            
            current_vel = np.array([current_odometry.twist.twist.linear.x,
                                   current_odometry.twist.twist.linear.y, 0])
            current_w = np.array([0, 0, current_odometry.twist.twist.angular.z])
            
            port_vel = current_vel + np.cross(current_w, self.port_vector)
            port_angle = np.arctan2(port_vel[1], port_vel[0])
            port_wheel_vel = np.linalg.norm(port_vel)
            
            starboard_vel = current_vel + np.cross(current_w, self.starboard_vector)
            starboard_angle = np.arctan2(starboard_vel[1], starboard_vel[0])
            starboard_wheel_vel = np.linalg.norm(starboard_vel)
        
            stern_vel = current_vel + np.cross(current_w, self.stern_vector)
            stern_angle = np.arctan2(stern_vel[1], stern_vel[0])
            stern_wheel_vel = np.linalg.norm(stern_vel)

            port_angle, port_wheel_vel = self.mirror_pod(port_angle, port_wheel_vel)
            starboard_angle, starboard_wheel_vel = self.mirror_pod(starboard_angle, starboard_wheel_vel)
            stern_angle, stern_wheel_vel = self.mirror_pod(stern_angle, stern_wheel_vel)
            
        else:
            
            port_angle = 0
            port_wheel_vel = 0
            
            starboard_angle = 0
            starboard_wheel_vel = 0
            
            stern_angle = 0
            stern_wheel_vel = 0
            
        joints.header = header
            
        joints.name.append("port_steering_joint")
        joints.position.append(port_angle)
        joints.velocity.append(0)

        joints.name.append("port_axle")
        joints.position.append(0)
        joints.velocity.append(port_wheel_vel)

        joints.name.append("starboard_steering_joint")
        joints.position.append(starboard_angle)
        joints.velocity.append(0)

        joints.name.append("starboard_axle")
        joints.position.append(0)
        joints.velocity.append(starboard_wheel_vel)

        joints.name.append("stern_steering_joint")
        joints.position.append(stern_angle)
        joints.velocity.append(0)

        joints.name.append("stern_axle")
        joints.position.append(0)
        joints.velocity.append(stern_wheel_vel)

        joints.name.append("carousel_joint")
        joints.position.append(0);
        joints.velocity.append(0);

        self.joint_state_pub.publish(joints)
    
    def mirror_pod(self, angle, velocity):
        steering_max = math.pi/2
        steering_min = -math.pi/2
        
        if (angle>steering_max):
            angle -= math.pi
            velocity *= -1.0
            
        if (angle<steering_min):
            angle += math.pi
            velocity *= -1.0    
        
        return angle, velocity
   
#dummy manipulator state, waits 10 seconds and
#exits with success unless preempted
class ManipulatorState(smach.State):
    def __init__(self, set_success):
        smach.State.__init__(self,
                             outcomes=['succeeded','preempted'],
                             input_keys=['action_goal','action_result'],
                             output_keys=['action_goal','action_result'])
        
        self.set_success = set_success
    
    def execute(self, userdata):
        start_time = rospy.get_time()
        #wait then return success
        if userdata.action_goal.type == manipulator_msg.ManipulatorGoal.HOME:
            wait_time = 1.0
        else:
            wait_time = 5.0
        
        while (rospy.get_time() - start_time) < wait_time:
            if self.preempt_requested():
                userdata.action_result = manipulator_msg.ManipulatorResult('fake_failure', False)
                self.service_preempt()
                return 'preempted'
            rospy.sleep(0.2)
        userdata.action_result = manipulator_msg.ManipulatorResult('fake_success', True)
        if userdata.action_goal.type != manipulator_msg.ManipulatorGoal.HOME:
            self.set_success()
        return 'succeeded'
    



       
