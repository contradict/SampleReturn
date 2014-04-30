import rospy
import math
import rosgraph_msgs.msg as rosgraph_msg
from collections import namedtuple
from samplereturn_msgs.msg import VoiceAnnouncement

import geometry_msgs.msg as geometry_msg
import actionlib_msgs.msg as action_msg


actionlib_working_states = [action_msg.GoalStatus.ACTIVE, action_msg.GoalStatus.PENDING]

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

def get_current_robot_pose(tf_listener):
    tf_listener.waitForTransform('/map',
                                 '/base_link',
                                 rospy.Time(0),
                                 rospy.Duration(1.0))
    
    now = tf_listener.getLatestCommonTime('/map', '/base_link')
    
    pos, quat = tf_listener.lookupTransform('/map',
                                            '/base_link',
                                            rospy.Time(0))
    
    header = std_msg.Header(0, now, '/map')
    
    pose = geometry_msg.PoseStamped(header,
                                    geometry_msg.Pose(geometry_msg.Point(*pos),
                                                      geometry_msg.Quaternion(*quat)
                                                      )
                                    )
    return pose


