import matplotlib.pyplot as plt
import rospy
import cv_bridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from stereo_msgs.msg import DisparityImage
import numpy as np
import cv2
import image_geometry
import sensor_msgs.point_cloud2 as pc2

global images

def makesub(topic, name, br, type=Image):
    global images
    images={}
    def saveit(msg):
        global images
        images[name]=br(msg)
    return rospy.Subscriber(topic, type, saveit)

def makesubs():
    br = cv_bridge.CvBridge()
    s1 = makesub("left/rect_mono", "left", br.imgmsg_to_cv2)
    sc = makesub("left/rect_color", "left_color", br.imgmsg_to_cv2)
    s2 = makesub("right/rect_mono", "right", br.imgmsg_to_cv2)
    dbr = lambda x:br.imgmsg_to_cv2(x.image)
    s3 = makesub("disparity", "disparity", dbr, type=DisparityImage)
    lsub = makesub("left/camera_info", "left_info", lambda msg: msg,
            type=CameraInfo)
    rsub = makesub("right/camera_info", "right_info", lambda msg: msg,
            type=CameraInfo)
    return lsub, rsub, s1, sc, s2, s3


def unsub(ss):
    for s in ss:
        s.unregister()

def refine(disparity, left, right, blocksize=7, maxdisp=64):
    refined_disparity = np.copy(disparity)
    left = cv2.Sobel(left, cv2.CV_32F, 1, 0, 3)
    right = cv2.Sobel(right, cv2.CV_32F, 1, 0, 3)
    ssd = np.zeros_like(disparity)
    ssdm = np.zeros_like(disparity)
    ssdp = np.zeros_like(disparity)
    for row in xrange(blocksize, disparity.shape[0]-blocksize):
        for col in xrange(blocksize+maxdisp, disparity.shape[1]-blocksize-1):
            disp = disparity[row, col]
            lbl = col-blocksize
            rbl = lbl-disp
            lbr = col+blocksize+1
            rbr = lbr-disp
            bt = max(0, row-blocksize)
            bb = min(left.shape[0], row+blocksize+1)
            leftblock = left[bt:bb, lbl:lbr]
            try:
                ssd[row, col] = ((leftblock - right[bt:bb, rbl:rbr])**2).sum()
                ssdp[row, col] = ((leftblock - right[bt:bb, rbl+1:rbr+1])**2).sum()
                ssdm[row, col] = ((leftblock - right[bt:bb, rbl-1:rbr-1])**2).sum()
            except ValueError, e:
                print row, col, disp
                print lbl, lbr, bt, bb
                print rbl, rbr
                print leftblock.shape
                print e
                return
            a = (ssdm[row, col]+ssdp[row, col])/2. - ssd[row, col]
            b = (ssdp[row, col]-ssdm[row, col])/2.
            d = a+abs(b)
            if a > 0 and abs(d) > 1:
                refined_disparity[row, col] += b/d
    return refined_disparity, ssd, ssdp, ssdm

def reproject(disparity):
    global models
    mdl = image_geometry.StereoCameraModel()
    mdl.fromCameraInfo(images['left_info'], images['right_info'])
    pts = cv2.reprojectImageTo3D(disparity, mdl.Q)
    return pts

def create_colored_cloud(pts, color):
    ptsimage = pts.reshape((-1, 3))
    rgb = color.reshape((-1, 3))
    argb = np.c_[rgb, np.zeros(shape=(rgb.shape[0], 1), dtype='u1')]
    pcpack = np.ndarray(buffer=argb, dtype='f4', shape=(argb.shape[0], 1))
    pc = pc2.create_cloud(images['left_info'].header,
            [pc2.PointField("x", 0, pc2.PointField.FLOAT32, 1),
             pc2.PointField("y", 4, pc2.PointField.FLOAT32, 1),
             pc2.PointField("z", 8, pc2.PointField.FLOAT32, 1),
             pc2.PointField("rgb", 12, pc2.PointField.FLOAT32, 1)],
            np.c_[ptsimage, pcpack])
    return pc


def pcpub(topic='pointcloud'):
    return rospy.Publisher(topic, PointCloud2, queue_size=1)

def setup():
    rospy.init_node("ipy")
    subs = makesubs()
    ppub = pcpub()
    return subs, ppub

def step(ppub):
    global images
    rd, ssd, ssdm, ssdp = refine(images['disparity'], images['left'],
            images['right'], blocksize=10, maxdisp=48)
    pts = reproject(rd)
    pc = create_colored_cloud(pts, images['left_color'])
    ppub.publish(pc)
    return rd, ssd, ssdm, ssdp, pts, pc


