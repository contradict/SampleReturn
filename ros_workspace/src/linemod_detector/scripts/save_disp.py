import rospy
import cv2
import numpy as np
import scipy.misc
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge

def callback(msg):
  bridge = CvBridge()
  img = np.asarray(bridge.imgmsg_to_cv(msg.image,desired_encoding="passthrough"))
  scipy.misc.imsave("disp.jpg",img)

def listener():
  rospy.init_node('listener')
  rospy.Subscriber("/cameras/manipulator/disparity", DisparityImage, callback)
  rospy.spin()

if __name__ == '__main__':
  listener()
