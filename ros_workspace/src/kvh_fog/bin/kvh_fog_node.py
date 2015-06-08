#!/usr/bin/env python
import rospy
from math import sin, cos, radians
import numpy as np
import threading
from kvh_fog import driver
from kvh_fog.srv import MeasureBiasResponse, MeasureBias
from sensor_msgs.msg import Imu

class KVHFOGNode(object):
    def __init__(self):
        self._port = rospy.get_param("port", "/dev/ttyUSB0")
        self._discard_count = rospy.get_param("discard_count", 10)
        self._reopen_delay = rospy.get_param("reopen_delay", 30)
        self._frame_id = rospy.get_param("frame_id", "gyro")
        # DSP3000 async has 100Hz update rate
        self._sample_period = rospy.get_param("sample_period", 0.010)
        self._sigma_omega = rospy.get_param("sigma_omega",
                radians(0.0667)/3600.)
        # based on bias stability of 1 degree/hour
        self._sigma_theta = rospy.get_param("sigma_theta",
                radians(1.0)/3600.)
        lat = rospy.get_param("lattitude", 34.156406)
        self._rate_bias = driver.EARTH_RATE*sin(radians(lat))
        self._gyro = None
        self._seq = 0

        self._last_angle = None
        self._last_time = None
        self._averaging_start_time = None
        self._averaging_interval = None
        self._averaging_start_angle = None
        self._average_value = None
        self._average_count = None
        self._bias_measurement_time = None
        self._pub = rospy.Publisher("imu", Imu, queue_size=10)
        self._avg_srv = rospy.Service("measure_bias", MeasureBias,
                self.handle_averaging)
        self._runthread = threading.Thread(target=self.run)

    def start(self):
        self._runthread.start()

    def open(self):
        self._gyro = driver.DSP3000(self._port)

    def run(self):
        while not rospy.is_shutdown():
            try:
                self.open()
            except driver.SerialException, e:
                rospy.logerr("Error opening serial port: %s", e)
                rospy.logerr("Retry in %f.", self._reopen_delay)
                rospy.sleep(rospy.Duration(self._reopen_delay))
                continue
            self._read_gyro()

    def _read_gyro(self):
        to_discard = self._discard_count
        mode_set = False
        while not rospy.is_shutdown():
            try:
                now, valid, angle = self._gyro.read()
            except driver.SerialException, e:
                rospy.logerr("Error reading from device: %s", e)
                rospy.logerr("Retry in %f.", self._reopen_delay)
                self._gyro.close()
                rospy.sleep(rospy.Duration(self._reopen_delay))
                break
            if not valid:
                continue
            if not mode_set:
                self._gyro.set_integrated_mode()
                self._gyro.zero_integrator()
                mode_set = True
            if to_discard > 0:
                to_discard -= 1
                self._bias_measurement_time = now
                continue

            self._check_averaging(now, angle)

            self._send_one(now, angle)

    def _check_averaging(self, now, angle):
        self._last_angle = angle
        self._last_time = now
        if self._averaging_start_time is not None:
            dt = (now - self._averaging_start_time).to_sec()
            if (self._gyro.current_mode == self._gyro.MODE_RATE or
                    self._gyro.current_mode == self._gyro.MODE_DTHETA):
                self._average_value += angle
                self._average_count += 1
            if dt > self._averaging_interval:
                if self._gyro.current_mode == self._gyro.MODE_INTEGRATED:
                    self._rate_bias = (angle - self._averaging_start_angle)/dt
                if self._gyro.current_mode == self._gyro.MODE_RATE:
                    self._rate_bias = self._average_value/self._average_count
                if self._gyro.current_mode == self._gyro.MODE_DTHETA:
                    self._rate_bias = self._average_value/dt
                self._averaging_start_time = None

    def _send_one(self, now, angle):
        msg = Imu()
        msg.header.stamp = now
        msg.header.seq = self._seq
        self._seq += 1
        msg.header.frame_id = self._frame_id
        msg.angular_velocity_covariance = np.zeros(9)
        msg.orientation_covariance = np.zeros(9)
        if self._gyro.current_mode == self._gyro.MODE_RATE:
            msg.angular_velocity.z = angle - self._rate_bias
            msg.angular_velocity_covariance[8] = (self._sigma_omega*self._sample_period)**2
        if self._gyro.current_mode == self._gyro.MODE_DTHETA:
            msg.angular_velocity = angle/self._sample_period - self._rate_bias
            msg.angular_velocity_covariance[8] = (self._sigma_omega*self._sample_period)**2
        if self._gyro.current_mode == self._gyro.MODE_INTEGRATED:
            dt = (now - self._bias_measurement_time).to_sec()
            corrected_angle = angle - self._rate_bias*dt
            msg.orientation.w = cos(corrected_angle/2.0)
            msg.orientation.z = sin(corrected_angle/2.0)
            msg.orientation_covariance[8] = self._sigma_theta*self._sigma_theta

        self._pub.publish(msg)

    def handle_averaging(self, req):
        if self._last_angle is None or self._last_time is None:
            resp = MeasureBiasResponse(False, self._rate_bias)
            return resp
        self._averaging_interval = req.averaging_time
        self._averaging_start_angle = self._last_angle
        self._averaging_start_time = self._last_time
        while self._averaging_start_time is not None:
            if (rospy.Time.now()-self._averaging_start_time).to_sec()>2*req.averaging_time:
                resp = MeasureBiasResponse(False, self._rate_bias)
                return resp
            rospy.sleep(0.1)
        resp = MeasureBiasResponse(True, self._rate_bias)
        return resp

if __name__ == "__main__":
    rospy.init_node("kvh_fog")
    node = KVHFOGNode()
    node.start()
    rospy.spin()
