#!/usr/bin/python
import rospy
import sys
from samplereturn_msgs.msg import VoiceAnnouncement
from platform_motion.msg import GPIO
from std_msgs.msg import Float64

class VoltageAnnouncer(object):
    def __init__(self):
        rospy.init_node('voltage_announcer')
        self.servo_id = rospy.get_param("servo_id", 1)
        self.button_mask = rospy.get_param("button_mask", 4)
        self.lowVoltageLimit = rospy.get_param("lowVoltageLimit", 49)
        rospy.Subscriber("gpio_read", GPIO, self.gpio)
        rospy.Subscriber("battery_voltage", Float64, self.voltageCallBack)
        self.voltage = None
        self.audioPub=rospy.Publisher("audio_navigate", VoiceAnnouncement)
        self.whine_guard = False #flag to keep robot from whining about low voltage constantly
        rospy.spin()

    def gpio(self, gpio):
        if gpio.servo_id == self.servo_id:
            if ((gpio.previous_pin_states ^ gpio.new_pin_states) &
                    self.button_mask):
                self.sayVoltage()

    def voltageCallBack(self, float):
        self.voltage=float.data
        #rospy.logdebug("voltage_announcer received voltage %f", self.voltage)
        if ((self.voltage <= self.lowVoltageLimit) and not self.whine_guard):
            msg = VoiceAnnouncement()
            msg.priority = VoiceAnnouncement.FIRST
            msg.words = "Warning. Battery voltage, %4.1f"%self.voltage
            self.audioPub.publish(msg)
            self.whine_guard = True
            rospy.Timer(rospy.Duration(15.0), self.clear_guard, oneshot = True)

    def clear_guard(self, event):
        self.whine_guard = False
        
    def sayVoltage(self):
        msg = VoiceAnnouncement()
        msg.priority = VoiceAnnouncement.FIRST
        if self.voltage is not None:
            msg.words = "Battery voltage, %4.1f"%self.voltage
        else:
            msg.words = "No battery voltage reported"
        self.audioPub.publish(msg)

if __name__=="__main__":
    VoltageAnnouncer()
