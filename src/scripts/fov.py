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
    P = np.array(cam_info.P).reshape((3, 4))
    left = np.arctan(P[0, 2]/P[0, 0])
    right = np.arctan((cam_info.width-P[0, 2])/P[0, 0])
    top = np.arctan(P[1, 2]/P[1, 1])
    bottom = np.arctan((cam_info.height-P[1, 2])/P[1, 1])
    return left, right, top, bottom

def compute_stereo_fov(left_info, right_info, max_d, window):
    Pleft = np.array(left_info.P).reshape((3, 4))
    Pright = np.array(right_info.P).reshape((3, 4))
    fx = Pleft[0, 0]
    fy = Pleft[1, 1]
    cx = Pleft[0, 2]
    cy = Pleft[1, 2]
    Tx = -Pright[0, 3]
    left = np.arctan((cx-window/2.-max_d)/fx)
    right = np.arctan((left_info.width - window/2. - cx)/fx)
    top = np.arctan((cy-window/2.)/fy)
    bottom = np.arctan((left_info.height - cy - window/2.)/fy)
    z_min = Tx/max_d
    z_max = Tx/1.0
    return left, right, top, bottom, z_min, z_max

def main():
    if len(sys.argv)<2:
        print("%s calibration_file"%sys.argv[0])
        print("prints field of view for calibrated camera")
        exit(1)
    if len(sys.argv) == 2:
        inf = parse_yaml(sys.argv[1])
        l, r, t, b = compute_fov(inf)
    else:
        left = parse_yaml(sys.argv[1])
        right = parse_yaml(sys.argv[2])
        max_d = 48
        if len(sys.argv)>3:
            max_d = int(sys.argv[3])
        win_sz = 21
        if len(sys.argv)>4:
            win_sz = int(sys.argv[4])
        l, r, t, b, z_min, z_max = compute_stereo_fov(left, right,
                max_d, win_sz)
        print("Z range (%4.2f, %4.2f)"%(z_min, z_max))
    print("Horizontal: %4.1f"%np.degrees(l+r))
    print("Vertical:   %4.1f"%np.degrees(t+b))
    print("Left, right of center: (%4.1f, %4.1f)"%
            (np.degrees(l), np.degrees(r)))
    print("Above, below center:   (%4.1f, %4.1f)"%
            (np.degrees(t), np.degrees(b)))



if __name__ == "__main__":
    main()

