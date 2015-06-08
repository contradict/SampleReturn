import rospy
import serial
from serial import SerialException
import math

EARTH_RATE = math.radians(-15.04107/3600.)

def unwrap(theta):
    while theta > math.pi:
        theta -= 2*math.pi
    while theta < -math.pi:
        theta += 2*math.pi
    return theta

class DSP3000(object):
    MODE_RATE = "R"
    MODE_DTHETA = "A"
    MODE_INTEGRATED = "P"
    def __init__(self, portname, timeout=0.1):
        self.port = serial.Serial(portname, 38400, timeout=timeout)
        self.current_mode = None

    def set_rate_mode(self):
        """ Set output to angular rate
        """
        self._send("R")
        self.current_mode = "R"

    def set_dtheta_mode(self):
        """ Set output to incremental angle mode
        """
        self._send("A")
        self.current_mode = "A"

    def set_integrated_mode(self):
        """ Set output to integrated angle mode
        """
        self._send("P")
        self.current_mode = "P"

    def zero_integrator(self):
        """ Zero internal angle integrator
        """
        if self.current_mode != "P":
            raise Exception("Zero only valid in integrated angle mode")
        self._send("Z")

    def _send(self, message):
        """Send a string to the device.

        The manual indicates more than one send may be necessary,
        but gives no more details. Third time's the charm?
        """
        self.port.write(message)
        self.port.write(message)
        self.port.write(message)

    def read(self):
        """ read the next message from the device

        Potentially invalid or None if an error occurs
        """
        line = self.port.readline().split()
        now = rospy.Time.now()
        angle, valid = None, False
        if len(line) == 2:
            rawangle, rawvalid = line
            try:
                angle = math.radians(float(rawangle))
                valid = rawvalid == "1"
            except ValueError:
                rospy.logerr("Invalid angle value: %s", rawangle)
        return now, valid, angle

    def close(self):
        """ Attempt to release serial port """
        try:
            self.port.close()
        except:
            pass

    def __del__(self):
        self.close()

