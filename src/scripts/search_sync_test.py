#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge

rospy.init_node("search_sync_test")
Tstart = rospy.Time.now()

NavImages = []

NavWin = None
NavIm = None
SearchWin = None
SearchIm = None
Bridge = CvBridge()
Searchdt = 0
SearchT = 0
Latency = 0
NavLatency = 0
Navstamp = 0

def display(evt):
    global NavWin
    global NavIm
    global Navstamp
    global SearchWin
    global SearchIm
    global Searchdt
    global SearchT
    if NavWin is None:
        NavWin = cv2.namedWindow("nav", cv2.WINDOW_NORMAL)
    if NavIm is not None:
        rospy.loginfo("display nav")
        cv2.putText(NavIm,
                "latency: %f"%(NavLatency),
            (10, 30),
            cv2.FONT_HERSHEY_PLAIN,
            2.0,
            (255, 255, 255),
            thickness=3)
        cv2.putText(NavIm,
                "stamp: %f"%(Navstamp),
            (10, 60),
            cv2.FONT_HERSHEY_PLAIN,
            2.0,
            (255, 255, 255),
            thickness=3)
        cv2.imshow("nav", NavIm)
        NavIm = None

    if SearchWin is None:
        SearchWin = cv2.namedWindow("search", cv2.WINDOW_NORMAL)
    if SearchIm is not None:
        cv2.putText(SearchIm,
                "nav-search: %f  latency: %f"%(Searchdt,
                                               Latency),
            (100, 200),
            cv2.FONT_HERSHEY_PLAIN,
            10.0,
            (255, 255, 255),
            thickness=5)
        cv2.putText(SearchIm,
                "Search Stamp: %f"%(SearchT),
            (100, 400),
            cv2.FONT_HERSHEY_PLAIN,
            10.0,
            (255, 255, 255),
            thickness=5)
        rospy.loginfo("display search")
        cv2.imshow("search", SearchIm)
        SearchIm = None
    key = cv2.waitKey(1)
    if key == 32 and len(NavImages)>0:
        im = NavImages[-1]
        Navstamp = im.header.stamp.to_sec()
        NavIm = Bridge.imgmsg_to_cv2(im, desired_encoding="bgr8")

def store_nav(msg):
    global NavLatency
    NavImages.append(msg)
    NavLatency = (rospy.Time.now()-msg.header.stamp).to_sec()
    while len(NavImages) > 60:
        drop=NavImages.pop(0)

rospy.Subscriber("/cameras/navigation/center/left/image_color", Image, store_nav)
#rospy.Subscriber("/camera/image_raw", Image, store_nav)

def check_sync(msg):
    global NavIm
    global SearchIm
    global Searchdt
    global SearchT
    global Navstamp
    global Latency
    global NavLatency
    stamp = msg.header.stamp
    rospy.loginfo("Got search img %s", stamp-Tstart)
    rospy.loginfo("searching %s - %s", NavImages[0].header.stamp-Tstart,
            NavImages[-1].header.stamp-Tstart)
    closest_nav_idx = np.argmin([np.abs((x.header.stamp-stamp).to_sec())
        for x in NavImages])
    rospy.loginfo("closest idx %d", closest_nav_idx)
    closest_nav = NavImages[closest_nav_idx]
    SearchT = stamp.to_sec()
    Navstamp = closest_nav.header.stamp.to_sec()
    Searchdt = (closest_nav.header.stamp - stamp).to_sec()
    Latency = (rospy.Time.now()-stamp).to_sec()
    rospy.loginfo("min dt %f", Searchdt)
    SearchIm = Bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    NavIm = Bridge.imgmsg_to_cv2(closest_nav, desired_encoding="bgr8")

rospy.Subscriber("/cameras/search/image", Image, check_sync)

rospy.Timer(rospy.Duration(0.100), display)

rospy.spin()
