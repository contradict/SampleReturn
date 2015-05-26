#!/usr/bin/env python
import yaml
import sys
 
import rospy
 
from std_msgs.msg import String
import samplereturn_msgs.srv as samplereturn_srv
 
def send(yamlfile):
    recovery_service = rospy.ServiceProxy('/processes/executive/start_recovery',
                                          samplereturn_srv.RecoveryParameters,)
    rospy.sleep(1.0)
    with open(yamlfile) as f:
        yaml_params = f.read()
        req = samplereturn_srv.RecoveryParametersRequest(yaml = yaml_params)
        print ("Sending recovery parameters: {!s}".format(yaml_params))
        print recovery_service(req)
    rospy.sleep(1.0)
 
if __name__=="__main__":
    rospy.init_node('recovery_sender')
    send(sys.argv[1])
