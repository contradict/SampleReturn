#!/usr/bin/env pyth
import sys
import math
import rospy
import rosnode
import actionlib
import tf
import tf_conversions
import numpy as np
from numpy import trunc,pi,ones_like,zeros,linspace, eye, sin, cos, arctan2
from copy import deepcopy

import smach
import smach_ros

import samplereturn.util as util

import actionlib_msgs.msg as action_msg
import std_msgs.msg as std_msg
import sensor_msgs.msg as sensor_msg
import nav_msgs.msg as nav_msg
import nav_msgs.srv as nav_srv
import platform_motion_msgs.msg as platform_msg
import platform_motion_msgs.srv as platform_srv
import manipulator_msgs.msg as manipulator_msg
import geometry_msgs.msg as geometry_msg
import move_base_msgs.msg as move_base_msg
import visual_servo_msgs.msg as visual_servo_msg
import samplereturn_msgs.msg as samplereturn_msg
import samplereturn_msgs.srv as samplereturn_srv

import sensor_msgs.point_cloud2 as pc2

import dynamic_reconfigure.srv as dynsrv
import dynamic_reconfigure.msg as dynmsg

class RobotSimulator(object):
    
    def __init__(self, mode='level_one'):
    
        rospy.init_node("robot_simulator")
        
        #simulator state variables and constants            
        self.GPIO_LEVEL_ONE = 0x20
        self.GPIO_LEVEL_TWO = 0x08
        
        self.mode = mode
        self.cameras_ready = True
        self.paused = False
        self.publish_samples = True
        self.fake_sample = {'x':9, 'y':-23, 'name':'none'}
        
        self.motion_mode = platform_srv.SelectMotionModeRequest.MODE_ENABLE

        #topic and action names
        manipulator_action_name = "/motion/manipulator/grab_action"
        home_wheelpods_name = "/motion/wheel_pods/home"
        home_carousel_name = "/motion/carousel/home"
        enable_wheelpods_name = "/motion/wheel_pods/enable"
        enable_carousel_name = "/motion/carousel/enable"

        enable_manipulator_detector_name = "/processes/sample_detection/manipulator/enable"
        
        select_motion_name = "/motion/CAN/select_motion_mode"

        cam_status_list = ["/cameras/navigation/port/status",
                                "/cameras/navigation/center/status",
                                "/cameras/navigation/starboard/status",
                                "/cameras/manipulator/status",
                                "/cameras/search/status"]
        
        gpio_read_name = "/io/gpio_read"
        pause_state_name = "/io/pause_state"
        
        visual_servo_name = "/processes/visual_servo/servo_action"
        
        detected_sample_search_name = "/processes/sample_detection/search/point"
        detected_sample_manipulator_name = "/processes/sample_detection/manipulator/point"
        beacon_pose_name = "/processes/beacon_finder/beacon_pose"

        point_cloud_center_name = "/cameras/navigation/center/points2"
        point_cloud_port_name = "/cameras/navigation/port/points2"
        point_cloud_starboard_name = "/cameras/navigation/starboard/points2"


        #tf stuff
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        
        self.zero_translation = (0,0,0)
        self.zero_rotation = (0,0,0,1)
                                                  
        self.fake_robot_pose = self.initial_pose()
        self.fake_odometry = self.initial_odometry()
       
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

        #planner, odom and tf
        self.planner_sub = rospy.Subscriber('/motion/planner_command',
                                            geometry_msg.Twist,
                                            self.handle_planner)
        
        self.odometry_pub = rospy.Publisher('/motion/odometry',
                                            nav_msg.Odometry)
        
        rospy.Timer(rospy.Duration(0.1), self.broadcast_tf)        
        
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
        
        self.manipulator_detector_enable = rospy.Service(enable_manipulator_detector_name,
                                                         samplereturn_srv.Enable,
                                                         self.enable_manipulator_detector)
                
        #visual servo stuff
        self.visual_servo_server = actionlib.SimpleActionServer(visual_servo_name,
                                                           visual_servo_msg.VisualServoAction,
                                                           self.run_visual_servo_action,
                                                           False)
        self.visual_servo_server.start()
        
        self.points_center_pub = rospy.Publisher(point_cloud_center_name,
                                                 sensor_msg.PointCloud2)
        self.points_port_pub = rospy.Publisher(point_cloud_port_name,
                                                 sensor_msg.PointCloud2)
        self.points_starboard_pub = rospy.Publisher(point_cloud_starboard_name,
                                                 sensor_msg.PointCloud2)
        
        rospy.Timer(rospy.Duration(0.15), self.publish_point_cloud)        

        #map stuff
        print "Waiting for map server"
        rospy.wait_for_service('/processes/planner/static_map')
        self.get_map = rospy.ServiceProxy('/processes/planner/static_map', nav_srv.GetMap)
        self.global_map = self.get_map().map
        print("map info: " + str(self.global_map.info))

        #rospy.spin()
        
    def handle_planner(self, twist):
        
        self.fake_robot_pose, self.fake_odometry = \
            self.integrate_odometry(self.fake_robot_pose,
                                    self.fake_odometry,
                                    twist)

    def publish_point_cloud(self, event):
        now = event.current_real

        header = std_msg.Header(0, now, '/map')
        target_frame = '/navigation_center_left_camera'
        try:
            self.tf_listener.waitForTransform('/map',
                                              target_frame,
                                              now,
                                              rospy.Duration(0.5))
            current_pose = util.get_current_robot_pose(self.tf_listener)
            transform = self.tf_listener.asMatrix(target_frame, header)
            center_cloud = self.get_pointcloud2(self.global_map,
                                                current_pose.pose.position,
                                                target_frame,
                                                transform)
            
            self.points_center_pub.publish(center_cloud)        
 
            header = std_msg.Header(0, now, '/navigation_port_left_camera')
            empty_cloud = pc2.create_cloud_xyz32(header,[])
            self.points_port_pub.publish(empty_cloud)
            empty_cloud.header.frame_id = 'navigation_starboard_left_camera'
            self.points_starboard_pub.publish(empty_cloud)
        except ( tf.Exception, tf.LookupException, tf.ConnectivityException):
            pass
        
    def broadcast_tf(self, event):
        now = rospy.Time.now()
        
        self.tf_broadcaster.sendTransform(self.zero_translation,
                                          self.zero_rotation,
                                          now,
                                          '/odom',
                                          '/map')
        
        self.fake_robot_pose, self.fake_odometry = \
            self.integrate_odometry(self.fake_robot_pose, self.fake_odometry)
        
        trans = (self.fake_robot_pose.pose.position.x,
                       self.fake_robot_pose.pose.position.y,
                       self.fake_robot_pose.pose.position.z)
        
        rot = (self.fake_robot_pose.pose.orientation.x,
                    self.fake_robot_pose.pose.orientation.y,
                    self.fake_robot_pose.pose.orientation.z,
                    self.fake_robot_pose.pose.orientation.w)
        
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
        
        #also publish odometry
        self.odometry_pub.publish(self.fake_odometry)

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
 
    def enable_manipulator_detector(self, req):
        rospy.sleep(0.5)
        return req.state
        
    def service_motion_mode_request(self, req):
        rospy.sleep(0.2)
        if req.mode == platform_srv.SelectMotionModeRequest.MODE_QUERY:
            return self.motion_mode
        else:
            self.motion_mode = req.mode
            return self.motion_mode
    
    def service_enable_wheelpods(self, req):
        rospy.sleep(0.5)
        return req.state
            
    def service_enable_carousel(self, req):
        rospy.sleep(0.5)
        return req.state
 
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
        self.fake_sample['name'] = 'PRECACHED'
        
    def shutdown(self):
        rospy.signal_shutdown("Probably closed from terminal")
        
    def zero_robot(self):
        self.fake_robot_pose = self.initial_pose()

    def get_pointcloud2(self, grid, position, target_frame, transform, range=12.0):
        header =  std_msg.Header(0, rospy.Time.now(), target_frame)
        
        grid_np = np.array(grid.data, dtype='u1').reshape((grid.info.height,grid.info.width))
        origin = np.array((grid.info.origin.position.x,
                           grid.info.origin.position.y,
                           grid.info.origin.position.z))
        world2map = lambda x:trunc((x-origin)/grid.info.resolution)
        ll = world2map(np.r_[position.x-range, position.y-range, 0])
        ll = np.clip(ll, zeros((3,)), np.r_[grid.info.height-1,
            grid.info.width-1,0])
        ur = world2map(np.r_[position.x+range, position.y+range, 0])
        ur = np.clip(ur, zeros((3,)), np.r_[grid.info.height-1,
            grid.info.width-1,0])
    
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
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.orientation.w=1.0
        odom.pose.covariance = list(eye(6).reshape((36,)))
        odom.twist.covariance = list(eye(6).reshape((36,)))
        return odom
    
    def initial_pose(self):
        pose = geometry_msg.PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "odom"
        pose.pose.orientation.w = 1.0
        return pose
    
    def integrate_odometry(self, current_pose, current_odometry, twist=None):
        if twist is None:
            twist = geometry_msg.Twist()
            twist.angular.z = current_odometry.twist.twist.angular.z
            twist.linear.x = current_odometry.twist.twist.linear.x
            twist.linear.y = current_odometry.twist.twist.linear.z
        now = rospy.Time.now();
        dt = (now - current_pose.header.stamp).to_sec()
        yaw = 2*arctan2(current_pose.pose.orientation.z,
                        current_pose.pose.orientation.w)
        cy = cos(yaw)
        sy = sin(yaw)
        current_pose.pose.position.x += (cy*current_odometry.twist.twist.linear.x*dt
                                         -sy*current_odometry.twist.twist.linear.y*dt)
        current_pose.pose.position.y += (sy*current_odometry.twist.twist.linear.x*dt
                                         +cy*current_odometry.twist.twist.linear.y*dt)
        yaw += current_odometry.twist.twist.angular.z*dt
        current_pose.pose.orientation.w = cos(yaw/2)
        current_pose.pose.orientation.z = sin(yaw/2)
        current_pose.header.stamp = now
        current_odometry.pose.pose = deepcopy(current_pose.pose)
        current_odometry.twist.twist = deepcopy(twist)
        current_odometry.header.stamp = now
        return current_pose, current_odometry        
   
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
    



       
