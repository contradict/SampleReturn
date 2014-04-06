import rospy
import rosgraph_msgs.msg as rosgraph_msg
from collections import namedtuple
from samplereturn_msgs.msg import VoiceAnnouncement

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

