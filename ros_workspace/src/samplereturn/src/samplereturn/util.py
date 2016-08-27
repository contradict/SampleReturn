import rospy
import math
from copy import deepcopy
import tf
import numpy as np
import yaml

from collections import namedtuple

from samplereturn_msgs.msg import VoiceAnnouncement
import rosgraph_msgs.msg as rosgraph_msg
import std_msgs.msg as std_msg
import geometry_msgs.msg as geometry_msg
import actionlib_msgs.msg as action_msg
import platform_motion_msgs.srv as platform_srv
from sensor_msgs.msg import CameraInfo


actionlib_working_states = [action_msg.GoalStatus.ACTIVE,
                            action_msg.GoalStatus.PENDING]

class AnnouncerInterface(object):
    def __init__(self, topic):
        self.pub = rospy.Publisher(topic, VoiceAnnouncement, queue_size=10)
        self.msg = VoiceAnnouncement()
        
    def say(self, words):
        self.msg.priority = VoiceAnnouncement.LAST
        self.msg.words = words
        self.pub.publish(self.msg)
    
    def say_next(self, words):
        self.msg.priority = VoiceAnnouncement.FIRST
        self.msg.words = words
        self.pub.publish(self.msg)
    
    def say_now(self, words):
        self.msg.priority = VoiceAnnouncement.OVERRIDE
        self.msg.words = words
        self.pub.publish(self.msg)
        
    def audio_ready(self):
        return (self.pub.get_num_connections() > 0)
        
    def __getstate__(self):
        return {'Class':'AnnouncerInterface'}

def wait_for_rosout():
    test_pub = rospy.Publisher("/rosout", rosgraph_msg.Log, queue_size=1)
    while (test_pub.get_num_connections() < 1 and not rospy.is_shutdown()):
        rospy.sleep(rospy.Duration(0.1))
    test_pub.unregister()
    
def get_node_params():
    node_params_dict = rospy.get_param(rospy.get_name())
    ParamTuple =  namedtuple('ParamTuple', node_params_dict.keys())
    return ParamTuple(**node_params_dict)

#takes two std_msg.Point objects, and returns the xy plane distance
def point_distance_2d(pt1, pt2):
    return math.sqrt((pt1.x-pt2.x)**2 + (pt1.y-pt2.y)**2)

#returns the distance between point, and the line from pt1 to pt2
def point_distance_to_line(pt0, pt1, pt2):
    return( np.abs( (pt2.y-pt1.y)*pt0.x - (pt2.x-pt1.x)*pt0.y  + pt2.x*pt1.y - pt2.y*pt1.x ) /
            math.sqrt((pt2.x-pt1.x)**2 + (pt2.y-pt1.y)**2) )

#takes two std_msg.PoseStamped objects, and returns the xy plane distance
def pose_distance_2d(pose1, pose2):
    return math.sqrt((pose1.pose.position.x-pose2.pose.position.x)**2 + 
                     (pose1.pose.position.y-pose2.pose.position.y)**2)

#takes two Points and returns the quaternion that points from pt1 to pt2
def pointing_quaternion_2d(pt1, pt2):
    yaw = math.atan2(pt2.y - pt1.y, pt2.x - pt1.x)
    quat_vals = tf.transformations.quaternion_from_euler(0, 0, yaw)
    return geometry_msg.Quaternion(*quat_vals)

def pointing_yaw(pt1, pt2):
    return math.atan2(pt2.y - pt1.y, pt2.x - pt1.x)

#takes a stamped pose and returns a stamped pose translated by x and y
#in the base_link frame, returns in /odom by default, useful for translating
#the robot pose for making new goals
def translate_base_link(listener, start_pose, dx, dy, frame_id):
    rospy.logdebug('start pose: %s', start_pose)
    base_header = std_msg.Header(0, rospy.Time(0), 'base_link')
    base_point = geometry_msg.PointStamped(base_header,
                                           geometry_msg.Point(dx, dy, 0))
    listener.waitForTransform(frame_id,
                        'base_link',
                        start_pose.header.stamp,
                        rospy.Duration(2.0))
    new_point = listener.transformPoint(frame_id, base_point)
    rospy.logdebug('new point: %s %s', dx, dy)
    new_pose = geometry_msg.Pose(new_point.point, start_pose.pose.orientation)
    rospy.logdebug('new pose: %s', new_pose)
    return geometry_msg.PoseStamped(start_pose.header, new_pose)

#translates a stamped pose by yaw and distance, returns in same frame
def pose_translate_by_yaw(start_pose, distance, yaw):
        new_pose = deepcopy(start_pose)
        new_pose.pose.position.x = new_pose.pose.position.x + distance * math.cos(yaw)
        new_pose.pose.position.y = new_pose.pose.position.y + distance * math.sin(yaw)
        return new_pose

def pose_translate_by_quat(start_pose, distance, quat):
    quat_vals = (quat.x, quat.y, quat.z, quat.w)
    yaw = tf.transformations.euler_from_quaternion(quat_vals)[-1]
    new_pose = deepcopy(start_pose)
    new_pose.pose.position.x = new_pose.pose.position.x + distance * math.cos(yaw)
    new_pose.pose.position.y = new_pose.pose.position.y + distance * math.sin(yaw)
    return new_pose    


#rotates a stamped pose, returns in same frame  
def pose_rotate(start_pose, angle):
    #rospy.loginfo('start pose: %s', start_pose)
    goal_quat = tf.transformations.quaternion_from_euler(0, 0, angle)
    start_quat = (start_pose.pose.orientation.x,
                  start_pose.pose.orientation.y,
                  start_pose.pose.orientation.z,
                  start_pose.pose.orientation.w)
    
    new_quat = tf.transformations.quaternion_multiply(start_quat, goal_quat)
    new_orientation = geometry_msg.Quaternion(*new_quat)
    new_pose = geometry_msg.Pose(start_pose.pose.position, new_orientation)
    return geometry_msg.PoseStamped(start_pose.header, new_pose)    

def get_current_robot_pose(tf_listener, frame_id):
    now = tf_listener.getLatestCommonTime(frame_id, 'base_link')
    pos, quat = tf_listener.lookupTransform(frame_id,
                                            'base_link',
                                            now)
    header = std_msg.Header(0, now, frame_id)
    pose = geometry_msg.Pose(geometry_msg.Point(*pos),
                             geometry_msg.Quaternion(*quat))
                                    
    return geometry_msg.PoseStamped(header, pose)

#get robot yaw pointing to the specified frame origin
def get_robot_distance_to_origin(tf_listener, frame_id):
    robot = get_current_robot_pose(tf_listener, frame_id)
    return point_distance_2d(robot.pose.position,
                                geometry_msg.Point(0,0,0))

#get robot yaw in the specified frame
def get_current_robot_yaw(tf_listener, frame_id):
    now = tf_listener.getLatestCommonTime(frame_id, 'base_link')
    pos, quat = tf_listener.lookupTransform(frame_id,
                                            'base_link',
                                            now)
    return tf.transformations.euler_from_quaternion(quat)[-1]

#get robot yaw pointing to the specified frame origin
def get_robot_yaw_to_origin(tf_listener, frame_id):
    robot = get_current_robot_pose(tf_listener, frame_id)
    return pointing_yaw(robot.pose.position,
                        geometry_msg.Point(0,0,0))

#get robot yaw pointing from the specified frame origin
def get_robot_yaw_from_origin(tf_listener, frame_id):
    robot = get_current_robot_pose(tf_listener, frame_id)
    return pointing_yaw(geometry_msg.Point(0,0,0),
                        robot.pose.position)

# gets yaw and distance to point in base_link, for strafe moves to specific points
def get_robot_strafe(tf_listener, point):
    tf_listener.waitForTransform('base_link',
                                point.header.frame_id,
                                point.header.stamp,
                                rospy.Duration(2.0))
    point_in_base = tf_listener.transformPoint('base_link',
                                               point).point
    robot_origin = geometry_msg.Point(0,0,0)
    yaw = pointing_yaw(robot_origin, point_in_base)        
    distance = point_distance_2d(robot_origin, point_in_base)
    return yaw, distance

def unwind(ang):
    while ang > np.pi:
      ang -= 2*np.pi
    while ang < -np.pi:
      ang += 2*np.pi
    return ang

class CANInterface(object):
    def __init__(self):
        self.CAN_select_motion_mode = \
                rospy.ServiceProxy("CAN_select_motion_mode",
                platform_srv.SelectMotionMode)
        self.joystick_command=rospy.Publisher("joystick_command", geometry_msg.Twist, queue_size=1)
        self.planner_command=rospy.Publisher("planner_command", geometry_msg.Twist, queue_size=1)
        self.servo_command=rospy.Publisher("servo_command", geometry_msg.Twist, queue_size=1)
        self.search_lights = rospy.Publisher("search_lights", std_msg.Bool, queue_size=1)

    def select_mode(self, motion_mode):
        return self.CAN_select_motion_mode(motion_mode)
        
    def publish_joy_state(self, joy_state):
        self.joystick_command.publish(joy_state.get_twist())

    def set_search_lights(self, state):
        self.search_lights.publish(state)
        
    def publish_zero(self):
        t=geometry_msg.Twist()
        t.linear.x=0
        t.linear.y=0
        t.linear.z=0
        t.angular.x=0
        t.angular.y=0
        t.angular.z=0        
        self.joystick_command.publish(t)
        self.planner_command.publish(t)
        self.servo_command.publish(t)
    
def parse_camera_info(filename):
    stream = file(filename, 'r')
    calib_data = yaml.load(stream)
    cam_info = CameraInfo()
    cam_info.width = calib_data['image_width']
    cam_info.height = calib_data['image_height']
    cam_info.K = calib_data['camera_matrix']['data']
    cam_info.D = calib_data['distortion_coefficients']['data']
    cam_info.R = calib_data['rectification_matrix']['data']
    cam_info.P = calib_data['projection_matrix']['data']
    cam_info.distortion_model = calib_data['distortion_model']
    return cam_info

