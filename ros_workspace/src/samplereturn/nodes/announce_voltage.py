#!/usr/bin/python
import rospy
import sys
sys.path.append('/opt/ros/groovy/stacks/audio_common/sound_play/src/')
from sound_play.msg import SoundRequest
from platform_motion.msg import GPIO
from std_msgs.msg import Float64

class VoltageAnnouncer(object):
    def __init__(self):
        rospy.init_node('voltage_announcer')
        self.voice = rospy.get_param("voice", "kal_diphone")
        self.servo_id = rospy.get_param("servo_id", 1)
        self.button_mask = rospy.get_param("button_mask", 4)
        rospy.Subscriber("/gpio_read", GPIO, self.gpio)
        rospy.Subscriber("/battery_voltage", Float64, self.voltage)
        self.voltage = None
        self.audio_pub=rospy.Publisher("/audio/navigate", SoundRequest)
        rospy.spin()

    def gpio(self, gpio):
        if gpio.servo_id == self.servo_id:
            if ((gpio.previous_pin_states ^ gpio.new_pin_states) &
                    self.button_mask):
                self.say_voltage()

    def voltage(self, float):
        self.voltage=float.data

    def say_voltage(self):
        msg = SoundRequest()
        msg.sound = SoundRequest.SAY
        msg.command = SoundRequest.PLAY_ONCE
        if self.voltage is not None:
            msg.arg = "Battery voltage %4.1f"%self.voltage
        else:
            msg.arg = "No battery voltage reported"
        msg.arg2 = 'voice.select "%s"'%self.voice
        self.audio_pub.publish(msg)

if __name__=="__main__":
    VoltageAnnouncer()
