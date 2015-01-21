#!/usr/bin/env python

import rospy

class TestNode(object):
    
    def __init__(self):
        rospy.init_node("test_node")
               
        while not rospy.is_shutdown():
            rospy.loginfo("TEST_NODE LOGINFO 1")
            rospy.loginfo("TEST_NODE LOGINFO 2")
            rospy.loginfo("TEST_NODE LOGINFO 3")
            rospy.loginfo("TEST_NODE LOGINFO 4")
            rospy.loginfo("TEST_NODE LOGINFO 5")
            rospy.loginfo("TEST_NODE LOGINFO 6")
            rospy.loginfo("TEST_NODE LOGINFO 7")
            rospy.loginfo("TEST_NODE LOGINFO 8")
            rospy.loginfo("TEST_NODE LOGINFO 9")
            rospy.loginfo("TEST_NODE LOGINFO 10")
            rospy.loginfo("TEST_NODE LOGINFO 11")
            rospy.loginfo("TEST_NODE LOGINFO 12")
            rospy.sleep(1.0)

        rospy.spin()

if __name__=="__main__":
    TestNode()