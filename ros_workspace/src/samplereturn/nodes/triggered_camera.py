#!/usr/bin/env python
"""
Use GPIO to trigger a camera at a constant rate.

Listen to the resulting image topic and attach GPIO timestamps to the
corresponding images.
"""
import rospy
import yaml
import Queue

import std_msgs.msg as std_msg
from sensor_msgs.msg import Image, CameraInfo
from platform_motion_msgs.srv import GPIOServiceRequest, GPIOService
from photo.srv import SetConfig, SetConfigRequest
from platform_motion_msgs.msg import GPIO


class TriggeredCamera(object):
    """ Send triggers and handle images """
    def __init__(self):
        self.trigger_count = 0
        self.image_count = 0
        self.paused = True
        self.trigger_rate = rospy.get_param('~rate', 1.0)
        self.time_offset = rospy.get_param('~capture_delay', 0.0)
        self.gpio_servo_id = rospy.get_param('~gpio_servo_id', 1)
        self.gpio_pin = rospy.get_param('~gpio_pin', 2)
        self.pulse_width = rospy.get_param('~pulse_width', 0.1)
        self.error_timeout = rospy.get_param('~error_timeout', 5.0)
        self.queue_size_warning = rospy.get_param('~queue_size_warning', 3)
        self.restart_delay = rospy.get_param('~restart_delay', 5.0)
        self.frame_id = rospy.get_param('~frame_id', 'search_camera')
        calibration_name = rospy.get_param('~calib_file', None)
        if calibration_name is not None:
            rospy.logdebug("calibration_name: %s", calibration_name)
            try:
                self.info = self.parse_yaml(calibration_name)
                self.info.header.frame_id = self.frame_id
                self.info_pub = rospy.Publisher('timestamped_info',
                        CameraInfo, queue_size=1)
            except IOError, exc:
                rospy.logerr(
            "Unable to open calibration file %s: %s, not publishing CameraInfo",
                calibration_name, exc)
        else:
            rospy.logwarn(
                    "No calibration file specified, not publishing CameraInfo")
            self.info = None

        self.timestamp_queue = Queue.Queue()

        self.output = rospy.Publisher('timestamped_image', Image, queue_size=1)
        self.status_pub = rospy.Publisher('cam_status',
                std_msg.String, queue_size=1)
        self.gpio_service = rospy.ServiceProxy('gpio_service', GPIOService,
                persistent=True)
        self.photo_config = rospy.ServiceProxy('set_config', SetConfig,
                persistent=True)

        rospy.Subscriber("triggered_image", Image, self.handle_image,
                queue_size=2, buff_size=2*5000*4000)
        rospy.Subscriber('pause_state', std_msg.Bool, self.handle_pause)

        self.error_status_timer = None
        self.start_trigger_timer(None)

    def handle_pause(self, msg):
        self.paused = msg.data

    def handle_image(self, msg):
        if self.error_status_timer is not None:
            self.error_status_timer.shutdown()
            self.error_status_timer = None
        self.image_count += 1
        rospy.logdebug("image seq: %d queue: %d", self.image_count,
                self.timestamp_queue.qsize())
        try:
            seq, stamp = self.timestamp_queue.get_nowait()
        except Queue.Empty:
            if self.pause_for_restart is not None:
                self.pause_for_restart.shutdown()
                self.start_restart_timer()
            rospy.logerr("Received image without sending a trigger")
            return
        self.status_pub.publish("Ready")
        msg.header.stamp = stamp
        msg.header.seq = seq
        msg.header.frame_id = self.frame_id
        self.output.publish(msg)
        if self.info is not None:
            self.info.header.stamp = stamp
            self.info.header.seq = seq
            self.info_pub.publish(self.info)
        qsz = self.timestamp_queue.qsize()
        if qsz > self.queue_size_warning:
            rospy.logwarn("trigger count: %d, image count: %d, queue size %d",
                    self.trigger_count, self.image_count, qsz)
            self.trigger_timer.shutdown()
            self.start_restart_timer()

    def start_restart_timer(self):
        self.clear_queue("Queue too deep, limit %d"%self.queue_size_warning)
        self.pause_for_restart = rospy.Timer(
                rospy.Duration(self.restart_delay),
                self.start_trigger_timer,
                oneshot=True)

    def start_trigger_timer(self, evt):
        rospy.loginfo("starting trigger")
        self.photo_config(SetConfigRequest(param="recordingmedia",
            value="1 SDRAM"))
        self.pause_for_restart = None
        self.trigger_timer = rospy.Timer(rospy.Duration(1./self.trigger_rate),
                self.trigger_camera)

    def trigger_camera(self, evt):
        if self.paused:
            if self.error_status_timer is not None:
                self.error_status_timer.shutdown()
                self.error_status_timer = None
            return
        self.trigger_count += 1
        rospy.logdebug("trigger seq: %d queue: %d", self.trigger_count,
                self.timestamp_queue.qsize())
        gpio_req = GPIOServiceRequest(GPIO(servo_id=self.gpio_servo_id,
                                   new_pin_states=0,
                                   pin_mask=self.gpio_pin))
        capture_stamp = self.gpio_service(gpio_req).stamp
        rospy.sleep(self.pulse_width)
        gpio_req.gpio.new_pin_states = self.gpio_pin
        self.gpio_service(gpio_req)
        capture_stamp += rospy.Duration(self.time_offset)
        self.timestamp_queue.put((self.trigger_count, capture_stamp))
        if self.error_status_timer is None:
            self.error_status_timer = rospy.Timer(
                    rospy.Duration(self.error_timeout),
                    lambda evt: self.clear_queue("Image receipt timeout."),
                    oneshot=True)

    def clear_queue(self, reason):
        rospy.logerr("Clearing timestamp queue with %d entries: %s",
            self.timestamp_queue.qsize(), reason)
        self.status_pub.publish("Error")
        while True:
            try:
                self.timestamp_queue.get_nowait()
            except Queue.Empty:
                break

    def parse_yaml(self, filename):
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

def main():
    rospy.init_node("triggered_camera")
    cam = TriggeredCamera()
    rospy.spin()

if __name__ == "__main__":
    main()
