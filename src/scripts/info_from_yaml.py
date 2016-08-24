import yaml
from sensor_msgs.msg import CameraInfo

def parse_yaml(filename):
    with open(filename) as stream:
        calib_data = yaml.load(stream)
    cam_info = CameraInfo()
    cam_info.width = calib_data['image_width']
    cam_info.height = calib_data['image_height']
    cam_info.K = calib_data['camera_matrix']['data']
    cam_info.D = calib_data['distortion_coefficients']['data']
    cam_info.R = calib_data['rectification_matrix']['data']
    cam_info.P = calib_data['projection_matrix']['data']
    cam_info.distortion_model = calib_data['distortion_model']
    #cam_info.binning_x = calib_data['binning_x']
    #cam_info.binning_y = calib_data['binning_y']
    #cam_info.roi.x_offset = calib_data['roi']['x_offset']
    #cam_info.roi.y_offset = calib_data['roi']['y_offset']
    #cam_info.roi.height = calib_data['roi']['height']
    #cam_info.roi.width = calib_data['roi']['width']
    #cam_info.roi.do_rectify = calib_data['roi']['do_rectify']
    
    return cam_info 
