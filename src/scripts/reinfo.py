#!/usr/bin/env python
import sys

import rosbag
from camera_info_manager import CameraInfoManager


topics={"/cameras/navigation/starboard/left/camera_info":
                {"name":"00b09d0100ca7fdb",
                 "url":"file:///home/russel/Desktop/SampleReturn/ros_workspace/src/samplereturn/calibration/ffmv-starboard-left-752.yaml",
                 "ns":"/cameras/navigation/starboard/left"},
        "/cameras/navigation/starboard/right/camera_info":
                {"name":"00b09d0100ca7fd5",
                 "url":"file:///home/russel/Desktop/SampleReturn/ros_workspace/src/samplereturn/calibration/ffmv-starboard-right-752.yaml",
                 "ns":"/cameras/navigation/starboard/right"},
        }

def load_info(topics):
    for t, s in topics.iteritems():
        manager = CameraInfoManager(s['name'],s['url'], s['ns'])
        manager.loadCameraInfo()
        assert( manager.isCalibrated() )
        s['info'] = manager.getCameraInfo()
        manager.svc.shutdown()

def replace_camera_info_messages(inbag, topics, outbag):
    for t, msg, ts in inbag.read_messages():
        if t in topics:
            camera_info = topics[t]['info']
            camera_info.header = msg.header
            outbag.write(t, camera_info, ts)
        else:
            outbag.write(t, msg, ts)


if __name__=="__main__":
    inbag = rosbag.Bag(sys.argv[1], 'r')
    outbag = rosbag.Bag(sys.argv[2], 'w')

    load_info(topics)
    replace_camera_info_messages(inbag, topics, outbag)

    inbag.close()
    outbag.close()
