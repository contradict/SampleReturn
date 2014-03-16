import rospy
import rosgraph_msgs.msg as rosgraph_msg
from collections import namedtuple

def wait_for_rosout():
    test_pub = rospy.Publisher("/rosout", rosgraph_msg.Log)
    while (test_pub.get_num_connections() < 1 and not rospy.is_shutdown()):
        rospy.sleep(rospy.Duration(0.1))
    test_pub.unregister()
    
def get_node_params():
    node_params_dict = rospy.get_param(rospy.get_name())
    ParamTuple =  namedtuple('ParamTuple', node_params_dict.keys())
    return ParamTuple(**node_params_dict)