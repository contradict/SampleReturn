#!/usr/bin/env python
import rospy
import numpy as np
import cv2

from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server

from sensor_msgs.msg import Image, CameraInfo
from samplereturn_msgs.msg import PatchArray, Patch

class PatchSaver(object):

    def __init__(self):
        rospy.init_node('patch_saver',log_level=rospy.DEBUG)

        self.patch_array_sub = rospy.Subscriber('patch_array', PatchArray,
                self.patch_array_callback, None, 1)

        self.debug_img_pub = rospy.Publisher('patch_array_debug_img', Image)

        self.bridge = CvBridge()

        self.path = rospy.get_param('~save_path')

    def patch_array_callback(self, PatchArray):
        rospy.loginfo("Patch Array Callback")
        num = np.random.random()
        if ((len(PatchArray.patch_array) > 1) and num > 0.9) or (num > 0.95):
            for idx, patch in enumerate(PatchArray.patch_array):
                image = np.asarray(self.bridge.imgmsg_to_cv2(patch.image,'bgr8'))
                mask = np.asarray(self.bridge.imgmsg_to_cv2(patch.mask,'mono8'))
                cv2.imwrite(self.path +
                        str(int(PatchArray.header.stamp.to_sec() * 1000))
                        + "_" + str(idx) + "_image.png", image)
                cv2.imwrite(self.path +
                        str(int(PatchArray.header.stamp.to_sec() * 1000))
                        + "_" + str(idx) + "_mask.png", mask)

if __name__=="__main__":
  try:
    PatchSaver()
    rospy.spin()
  except rospy.ROSInterruptException: pass
