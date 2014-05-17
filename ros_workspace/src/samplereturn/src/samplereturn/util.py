import rospy
import math
import tf

from collections import namedtuple

from samplereturn_msgs.msg import VoiceAnnouncement
import rosgraph_msgs.msg as rosgraph_msg
import std_msgs.msg as std_msg
import geometry_msgs.msg as geometry_msg
import actionlib_msgs.msg as action_msg
import platform_motion_msgs.msg as platform_msg
import platform_motion_msgs.srv as platform_srv


actionlib_working_states = [action_msg.GoalStatus.ACTIVE,
                            action_msg.GoalStatus.PENDING]

class AnnouncerInterface(object):
    def __init__(self, topic):
        self.pub = rospy.Publisher(topic, VoiceAnnouncement)
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
    test_pub = rospy.Publisher("/rosout", rosgraph_msg.Log)
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

#takes two std_msg.PoseStamped objects, and returns the xy plane distance
def pose_distance_2d(pose1, pose2):
    return math.sqrt((pose1.pose.position.x-pose2.pose.position.x)**2 + \
                     (pose1.pose.position.y-pose2.pose.position.y)**2)

#takes two Points and returns the quaternion that points from pt1 to pt2
def pointing_quaternion_2d(pt1, pt2):
    yaw = math.atan2(pt2.y - pt1.y, pt2.x - pt1.x)
    quat_vals = tf.transformations.quaternion_from_euler(0, 0, yaw)
    return geometry_msg.Quaternion(*quat_vals)

def pointing_yaw(pt1, pt2):
    return math.atan2(pt2.y - pt1.y, pt2.x - pt1.x)

#takes a stamped pose and returns a stamped pose distance forward in /map frame
def pose_translate(listener, start_pose, dx, dy):
    rospy.loginfo('start pose: %s', start_pose)
    header = std_msg.Header(0, rospy.Time(0), '/base_link')
    new_base_point = geometry_msg.PointStamped(header,
                                               geometry_msg.Point(dx, dy, 0))
    new_point = listener.transformPoint('/map', new_base_point)
    rospy.loginfo('new point: %s %s', dx, dy)
    new_pose = geometry_msg.Pose(new_point.point, start_pose.pose.orientation)
    header = std_msg.Header(0, rospy.Time(0), '/map')
    rospy.loginfo('new pose: %s', new_pose)
    return geometry_msg.PoseStamped(header, new_pose)    
    
def pose_rotate(listener, start_pose, angle):
    #rospy.loginfo('start pose: %s', start_pose)
    goal_quat = tf.transformations.quaternion_from_euler(0,0, angle)
    start_quat = (start_pose.pose.orientation.x,
                  start_pose.pose.orientation.y,
                  start_pose.pose.orientation.z,
                  start_pose.pose.orientation.w)
    
    new_quat = tf.transformations.quaternion_multiply(start_quat, goal_quat)
    new_orientation = geometry_msg.Quaternion(*new_quat)
    new_pose = geometry_msg.Pose(start_pose.pose.position, new_orientation)
    header = std_msg.Header(0, rospy.Time(0), '/map')
    #rospy.loginfo('rotate %f', angle)
    #rospy.loginfo('new pose: %s', new_pose)
    return geometry_msg.PoseStamped(header, new_pose)    


def get_current_robot_pose(tf_listener):
    now = tf_listener.getLatestCommonTime('/map', '/base_link')
    pos, quat = tf_listener.lookupTransform('/map',
                                            '/base_link',
                                            rospy.Time(0))
    header = std_msg.Header(0, now, '/map')
    pose = geometry_msg.Pose(geometry_msg.Point(*pos),
                             geometry_msg.Quaternion(*quat))
                                    
    return geometry_msg.PoseStamped(header, pose)

class CANInterface(object):
    def __init__(self):
        self.CAN_select_motion_mode = \
                rospy.ServiceProxy("CAN_select_motion_mode",
                platform_srv.SelectMotionMode)
        self.joystick_command=rospy.Publisher("joystick_command", geometry_msg.Twist)
        self.planner_command=rospy.Publisher("planner_command", geometry_msg.Twist)
        self.servo_command=rospy.Publisher("servo_command", geometry_msg.Twist)

    def select_mode(self, motion_mode):
        return self.CAN_select_motion_mode(motion_mode)
        
    def publish_joy_state(self, joy_state):
        self.joystick_command.publish(joy_state.get_twist())
        
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
    

