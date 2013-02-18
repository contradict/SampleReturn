#!/usr/bin/python
import rospy
import sys
sys.path.append('/opt/ros/groovy/stacks/audio_common/sound_play/src/')
from sound_play.msg import SoundRequest
from platform_motion.msg import GPIO
from platform_motion.srv import Enable

class PauseSwitch(object):
    def __init__(self):
        rospy.init_node('pause_switch')
        self.voice = rospy.get_param("voice", "kal_diphone")
        self.servo_id = rospy.get_param("servo_id", 1)
        self.button_mask = rospy.get_param("button_mask", 2)
        rospy.Subscriber("/gpio_read", GPIO, self.gpio)
        self.audio_pub=rospy.Publisher("/audio/navigate", SoundRequest)
        self.enable_service = rospy.ServiceProxy('enable_wheel_pods', Enable)
        self.paused = None
        rospy.spin()

    def gpio(self, gpio):
        if gpio.servo_id == self.servo_id:
            # GPIO is active low, so these are reversed
            if (gpio.new_pin_states & self.button_mask):
                self.pause(False)
            else:
                self.pause(True)

    def pause(self, state):
        if self.paused == state:
            return
        try:
            self.enable_service(state)
            if state:
                self.say("Pause switch enabled")
            else:
                self.say("Pause switch disabled")
            self.paused = state
        except rospy.ServiceException, e:
            rospy.logerr("Unable to use wheelpod enable service: %s", e)
            self.say("Pause switch failed")

    def say(self, utterance):
        msg = SoundRequest()
        msg.sound = SoundRequest.SAY
        msg.command = SoundRequest.PLAY_ONCE
        msg.arg = utterance
        msg.arg2 = 'voice.select "%s"'%self.voice
        self.audio_pub.publish(msg)

if __name__=="__main__":
    PauseSwitch()
