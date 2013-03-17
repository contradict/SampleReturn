#!/usr/bin/env python
import roslib; roslib.load_manifest('samplereturn')
from std_msgs.msg import Float64
from sensor_msgs.msg import CameraInfo
import rospy


class image_desync(object):
    def __init__(self):
        self.timestamps={'left':[], 'right':[]}
        self.pub = rospy.Publisher('desync', Float64)
        self.left_sub = rospy.Subscriber('left/camera_info', CameraInfo,
                lambda info, name='left': self.info_callback(name, info))
        self.right_sub = rospy.Subscriber('right/camera_info', CameraInfo,
                lambda info, name='right': self.info_callback(name, info))


    def info_callback(self, name, info):
        self.timestamps[name].append(info.header.stamp)
        if len(self.timestamps[name]) >2:
           self.timestamps[name].pop(0)
        if name=='left' and all([len(x)==2 for x in self.timestamps.itervalues()]):
            delta1 = (self.timestamps['left'][0] -
                    self.timestamps['right'][1]).to_sec()
            delta2 = (self.timestamps['left'][1] -
                self.timestamps['right'][1]).to_sec()
            if abs(delta1)<abs(delta2):
                delta = delta1
            else:
                delta = delta2
            self.pub.publish(Float64(delta))

if __name__=="__main__":
    rospy.init_node('image_desync')
    id = image_desync()
    rospy.spin()


