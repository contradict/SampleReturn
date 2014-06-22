import os
import rospy
from cv_bridge import CvBridge
import cv2
from samplereturn.simple_motion import SimpleMover
from sensor_msgs.msg import Image
import numpy as np

_cv_bridge = CvBridge()
save=False
def save_image_message(image, basename):
    global save
    if not save:
        return
    image_cv = _cv_bridge.imgmsg_to_cv2(image)
    time = int(image.header.stamp.to_sec()*1000)
    fname="%s_%d.jpg"%(basename, time)
    rospy.loginfo("Save %s", fname)
    cv2.imwrite(fname, image_cv)

def dospin(mover, angle):
    global save
    mover.execute_spin(np.sign(angle)*0.01, max_velocity=0.05)
    save=True
    mover.execute_spin(angle, max_velocity=0.05)
    save=False

def initialize():
    global save
    rospy.init_node("timetest")

    dslrbase="dslr_images/image"
    centerbase="center_images/image"

    rospy.Subscriber("/cameras/navigation/center/left/image_rect_color",
            Image,
            lambda image:save_image_message(image, centerbase))

    rospy.Subscriber("/cameras/search/image",
            Image,
            lambda image:save_image_message(image, dslrbase))

    mover = SimpleMover("")

    return mover
