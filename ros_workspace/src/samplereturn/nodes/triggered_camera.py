#!/usr/bin/env python
"""
Use GPIO to trigger a camera at a constant rate.

Simple description of strategy:
Listen to the resulting image topic and attach GPIO timestamps to the
corresponding images.

Timeuts:
    No image received in error_timeut - clear queue, wait for restart_delay
        with no images, the restart capture
    After sending first trigger - wait for first image
        If not received within error_timeout, clear queue, send another trigger
        and restart waiting for first image
        When image is received, stop first image timer and start regular
        triggering

If queue gets deeper than queue_size_warning, stop triggering, clear queue, wait
restart_delay with no images, then restart first trigger.
"""
import rospy
import yaml
import Queue

import std_msgs.msg as std_msg
from sensor_msgs.msg import Image, CameraInfo
from platform_motion_msgs.srv import GPIOServiceRequest, GPIOService, Enable, EnableResponse
from photo.srv import SetConfig, SetConfigRequest
from platform_motion_msgs.msg import GPIO


class TriggeredCamera(object):
    """ Send triggers and handle images """
    def __init__(self):
        self.trigger_count = 0
        self.image_count = 0
        self.enabled = True
        self.paused = False
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

        self.missing_image_timer = None
        self.pause_for_restart_timer = None
        self.wait_for_first_image_timer = None
        self.trigger_timer = None

        self.enable_srv = rospy.Service('enable', Enable, self.enable_callback)
        rospy.Subscriber("triggered_image", Image, self.handle_image,
                queue_size=2, buff_size=2*5000*4000)
        rospy.Subscriber('pause_state', std_msg.Bool, self.handle_pause)

        self.wait_for_service('gpio_service')
        self.wait_for_service('set_config')
        self.gpio_service = rospy.ServiceProxy('gpio_service', GPIOService,
                persistent=True)
        self.photo_config = rospy.ServiceProxy('set_config', SetConfig,
                persistent=True)

        self.start_trigger_timer(None)

    def wait_for_service(self, name):
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service(name, 20.0)
                break
            except rospy.ROSException:
                rospy.logerr("Cannot contact %s, retrying.", name)
            except rospy.ROSInterrupException:
                rospy.logerr("Never found %s, exiting for shutdown.", name)
                return

    def handle_pause(self, msg):
        self.paused = msg.data

    def handle_image(self, msg):
        if self.missing_image_timer is not None:
            self.missing_image_timer.shutdown()
            self.missing_image_timer = None

        if self.wait_for_first_image_timer is not None:
            self.wait_for_first_image_timer.shutdown()
            self.wait_for_first_image_timer = None
            rospy.loginfo("Got first image, beginning regular trigger.")
            self.trigger_timer = rospy.Timer(
                    rospy.Duration(1./self.trigger_rate),
                    self.trigger_camera)

        if self.pause_for_restart_timer is not None:
            self.start_restart_timer()

        self.image_count += 1
        rospy.logdebug("image seq: %d queue: %d", self.image_count,
                self.timestamp_queue.qsize())
        try:
            seq, stamp = self.timestamp_queue.get_nowait()
        except Queue.Empty:
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
            self.clear_queue("Queue too deep, limit %d"%self.queue_size_warning)
            self.start_restart_timer()

    def enable_callback(self, req):
        self.enabled = req.state
        resp = EnableResponse(self.enabled)
        return resp

    def start_restart_timer(self):
        if self.pause_for_restart_timer is not None:
            self.pause_for_restart_timer.shutdown()
        self.pause_for_restart_timer = rospy.Timer(
                rospy.Duration(self.restart_delay),
                self.start_trigger_timer,
                oneshot=True)

    def start_trigger_timer(self, evt):
        if self.pause_for_restart_timer is not None:
            self.pause_for_restart_timer.shutdown()
            self.pause_for_restart_timer = None
        rospy.loginfo("starting trigger")
        self.wait_for_service('set_config')
        self.photo_config(SetConfigRequest(param="recordingmedia",
            value="1 SDRAM"))
        self.send_one_trigger()
        self.wait_for_first_image_timer = rospy.Timer(
                rospy.Duration(2*self.error_timeout),
                self.no_first_image,
                oneshot=True)

    def no_first_image(self, evt):
        self.clear_queue("First trigger sent, no image received, retrying.")
        self.wait_for_first_image_timer = None
        self.start_trigger_timer(None)

    def send_one_trigger(self):
        self.trigger_count += 1
        gpio_req = GPIOServiceRequest(GPIO(servo_id=self.gpio_servo_id,
                                   new_pin_states=0,
                                   pin_mask=self.gpio_pin))
        try:
            capture_stamp = self.gpio_service(gpio_req).stamp
        except rospy.ServiceException:
            rospy.logerr("gpio service low failed, reconnecting.")
            self.wait_for_service('gpio_service')
            return

        rospy.sleep(self.pulse_width)
        gpio_req.gpio.new_pin_states = self.gpio_pin
        try:
            self.gpio_service(gpio_req)
        except rospy.ServiceException:
            rospy.logerr("gpio service high failed, reconnecting.")
            self.wait_for_service('gpio_service')
            # this probably resulted in a trigger, keep going!
        capture_stamp += rospy.Duration(self.time_offset)
        self.timestamp_queue.put((self.trigger_count, capture_stamp))

    def trigger_camera(self, evt):
        if not self.enabled or self.paused:
            if self.missing_image_timer is not None:
                self.missing_image_timer.shutdown()
                self.missing_image_timer = None
            return
        self.send_one_trigger()
        rospy.logdebug("trigger seq: %d queue: %d", self.trigger_count,
                self.timestamp_queue.qsize())
        if self.missing_image_timer is None:
            self.missing_image_timer = rospy.Timer(
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
