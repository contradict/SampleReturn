#!/usr/bin/env python
import roslib
import rospy
import rospkg
import numpy as np
import cv2
from copy import deepcopy

from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion
from tf import TransformListener
from dynamic_reconfigure.server import Server
from saliency_detector.cfg import close_range_detector_paramsConfig

from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image, CameraInfo
from samplereturn_msgs.msg import NamedPoint
from samplereturn_msgs.srv import Verify, VerifyResponse

# Subscribe to dslr image and current pursuit target
# When asked, take target location and get it in img coords
# Extract window, find contour, compute shape and classify
# Emit yes or no

class CloseRangeDetector(object):

    def __init__(self):
        rospy.init_node('close_range_detector',log_level=rospy.DEBUG)

        self.img_sub = rospy.Subscriber('image', Image, self.image_callback, None, 1)
        self.cam_info_sub = rospy.Subscriber('cam_info', CameraInfo, self.cam_info_callback, None, 1)

        self.debug_img_pub = rospy.Publisher('close_detector_debug_img', Image)

        rospy.Service("close_range_verify",Verify,self.verify_callback)

        self.bridge = CvBridge()
        self.tf = TransformListener()
        self.cam_model = None
        self.last_img = None

        self.win_size = 200

        rospack = rospkg.RosPack()

        self.classifier = cv2.SVM()
        self.classifier.load(rospack.get_path('saliency_detector')+'/config/trained_svm')

        self.server = Server(close_range_detector_paramsConfig, self.config_callback)

    def config_callback(self, config, level):
        self.use_SVM = config["use_SVM_classifier"]
        self.min_perimeter_ratio = config["min_perimeter_ratio"]
        self.max_perimeter_ratio = config["max_perimeter_ratio"]
        self.min_area_ratio = config["min_area_ratio"]
        self.max_area_ratio = config["max_area_ratio"]
        self.min_defect_ratio = config["min_defect_ratio"]
        self.max_defect_ratio = config["max_defect_ratio"]
        return config

    def verify_callback(self, req):
        # When a point request is received, take the last image, transform the point
        # from odom to search_camera, project into pixel space, compute shape metrics
        if self.cam_model is None or self.last_img is None:
            rospy.logerr("Verification Request with no image or camera model")
            resp = VerifyResponse(False)
            return resp
        else:
            req.target.header.stamp = self.last_img.header.stamp
            search_camera_point = self.tf.transformPoint("search_camera",req.target)
            point_3d = np.array([search_camera_point.point.x,search_camera_point.point.y,\
                                search_camera_point.point.z])
            img_point = self.cam_model.project3dToPixel(point_3d)
            rospy.logdebug("Image Point: %f, %f",img_point[0],img_point[1])
            img = np.asarray(self.bridge.imgmsg_to_cv2(self.last_img,'rgb8'))
            lab = cv2.cvtColor(img, cv2.COLOR_RGB2LAB)
            win = self.extract_window(lab, img_point)
            range_a = np.max(win[...,1]) - np.min(win[...,1])
            range_b = np.max(win[...,0]) - np.min(win[...,0])
            if range_a > range_b:
                shape = self.compute_shape_metrics(win[...,1])
            else:
                shape = self.compute_shape_metrics(win[...,2])
            if self.use_SVM:
                ret = self.classifier.predict(shape)
                rospy.logdebug("SVM class %f",ret)
            else:
                ret = (self.min_perimeter_ratio<shape[0]<self.max_perimeter_ratio) and\
                (self.min_area_ratio<shape[1]<self.max_area_ratio) and\
                (self.min_defect_ratio<shape[2]<self.max_defect_ratio)
            if ret == -1.0 or ret == False:
                resp = VerifyResponse(False)
            else:
                resp = VerifyResponse(True)
            return resp

    def cam_info_callback(self, CameraInfo):
        self.cam_model = PinholeCameraModel()
        self.cam_model.fromCameraInfo(CameraInfo)

    def image_callback(self, Image):
        self.last_img = deepcopy(Image)

    def extract_window(self, image, img_point):
        # Take the image, grab the widow around the img_point, handling borders
        win = image[img_point[1]-self.win_size:img_point[1]+self.win_size,\
                img_point[0]-self.win_size:img_point[0]+self.win_size]
        return win

    def compute_shape_metrics(self, img):
        mid = (float(np.max(img))+float(np.min(img)))/2.
        thresh = cv2.threshold(img,mid,255,cv2.THRESH_BINARY)[1]
        self.debug_img_pub.publish(self.bridge.cv2_to_imgmsg(thresh,'mono8'))
        contours, hier = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        largest_area = 0
        largest_contour = 0
        for i in contours:
            area = cv2.contourArea(i)
            if area > largest_area:
                largest_area = area
                largest_contour = i
        hull = cv2.convexHull(largest_contour,returnPoints=False)
        hull_pts = cv2.convexHull(largest_contour)
        perimeter_ratio = cv2.arcLength(hull_pts,True)/cv2.arcLength(largest_contour,True)
        area_ratio = cv2.contourArea(largest_contour)/cv2.contourArea(hull_pts)
        defects = cv2.convexityDefects(largest_contour,hull)
        depth = np.max(defects[...,-1])/256.0
        rect = cv2.boundingRect(hull_pts)
        defect_ratio = depth/np.max(rect[2:])
        rospy.logdebug("Shape metrics: %f, %f, %f",perimeter_ratio,area_ratio,defect_ratio)
        rospy.logdebug("Contour Area: %f",cv2.contourArea(largest_contour))
        return np.array([perimeter_ratio,area_ratio,defect_ratio],dtype=np.float32)

if __name__=="__main__":
  try:
    CloseRangeDetector()
    rospy.spin()
  except rospy.ROSInterruptException: pass
