#!/usr/bin/env python
import sys
import yaml
import numpy as np
from sensor_msgs.msg import CameraInfo

def parse_yaml(filename):
    stream = file(filename, 'r')
    calib_data = yaml.load(stream)
    cam_info = CameraInfo()
    cam_info.width = calib_data['image_width']
    cam_info.height = calib_data['image_height']
    cam_info.K = calib_data['camera_matrix']['data']
    cam_info.D = calib_data['distortion_coefficients']['data']
    cam_info.R = calib_data['rectification_matrix']['data']
    cam_info.P = calib_data['projection_matrix']['data']
    cam_info.distortion_model = calib_data['distortion_model']
    return cam_info

def compute_fov(cam_info):
    K = np.array(cam_info.K).reshape((3, 3))
    left = np.arctan(K[0, 2]/K[0, 0])
    right = np.arctan((cam_info.width-K[0, 2])/K[0, 0])
    top = np.arctan(K[1, 2]/K[1, 1])
    bottom = np.arctan((cam_info.height-K[1, 2])/K[1, 1])
    return left, right, top, bottom

def main():
    if len(sys.argv)<2:
        print("%s calibration_file"%sys.argv[0])
        print("prints field of view for calibrated camera")
        exit(1)
    inf = parse_yaml(sys.argv[1])
    l,r,t,b = compute_fov(inf)
    print("Horizontal: %4.1f"%np.degrees(l+r))
    print("Vertical:   %4.1f"%np.degrees(t+b))
    print("Left, right of center: (%4.1f, %4.1f)"%
            (np.degrees(l), np.degrees(r)))
    print("Above, below center:   (%4.1f, %4.1f)"%
            (np.degrees(t), np.degrees(b)))

if __name__ == "__main__":
    main()

