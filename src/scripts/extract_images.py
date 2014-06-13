import sys
import rosbag
from cv_bridge import CvBridge
import cv2

topic = sys.argv[1]
inbagname = sys.argv[2]

cv_bridge = CvBridge()

t0=None
for t, msg, time in rosbag.Bag(inbagname):
    if topic == t:
	image_cv = cv_bridge.imgmsg_to_cv2(msg)
        if t0 is None:
            t0 = msg.header.stamp
        t = (msg.header.stamp-t0).to_sec()
        cv2.imwrite("search-%04d.png"%int(t), image_cv)


