#!/usr/bin/python
import rospy
import sys
sys.path.append('/opt/ros/groovy/stacks/audio_common/sound_play/src/')
from sound_play.msg import SoundRequest
from platform_motion.msg import GPIO, ServoStatus
from platform_motion.srv import Enable
from std_msgs.msg import Bool

class PauseSwitch(object):
    def __init__(self):
        self.voice = rospy.get_param("voice", "kal_diphone")
        self.gpio_servo_id = rospy.get_param("gpio_servo_id", 1)
        self.button_mask = rospy.get_param("button_mask", 2)
        self.carousel_servo_id = rospy.get_param("carousel_servo_id", 1)
        wheelpod_servo_ids_string = rospy.get_param("wheelpod_servo_ids",
                                                    "[2,3,4,5,6,7]")
        self.wheelpod_servo_ids =\
            [int(filter(lambda x: x.isdigit(), s)) for s in
                    wheelpod_servo_ids_string.split(',')]

        self.wheelpod_servo_status = dict([(x,None) for x in
            self.wheelpod_servo_ids])
        self.carousel_servo_status = None

        rospy.Subscriber("/gpio_read", GPIO, self.gpio)
        rospy.Subscriber("/status_word", ServoStatus, self.status_word)

        self.pause_pub = rospy.Publisher("pause_state", Bool)
        self.audio_pub = rospy.Publisher("/audio/navigate", SoundRequest)

        self.enable_wheelpods = rospy.ServiceProxy('/enable_wheel_pods', Enable)
        self.enable_carousel = rospy.ServiceProxy('/enable_carousel', Enable)

        self.paused = None

    def gpio(self, gpio):
        if gpio.servo_id == self.gpio_servo_id:
            self.pause((gpio.new_pin_states & self.button_mask) ==
                    self.button_mask)

    def status_word(self, status_word):
        if status_word.servo_id == self.carousel_servo_id:
            self.carousel_servo_status = "Operation Enabled" in status_word.status
        if status_word.servo_id in self.wheelpod_servo_ids:
            self.wheelpod_servo_status[status_word.servo_id] = \
                "Operation Enabled" in status_word.status
        rospy.logdebug("carousel: %s, wheelpods: %s", self.carousel_servo_status,
                self.wheelpod_servo_status)
    
    def say(self, utterance):
        msg = SoundRequest()
        msg.sound = SoundRequest.SAY
        msg.command = SoundRequest.PLAY_ONCE
        msg.arg = utterance
        msg.arg2 = 'voice.select "%s"'%self.voice
        self.audio_pub.publish(msg)

    def set_pause_state(self, state):
        self.paused = state
        self.pause_pub.publish(Bool(self.paused))

    def check_wheelpod_status(self, status):
        return all([x==status for x in self.wheelpod_servo_status.itervalues()])

    def announce_pause_state(self):
        if self.paused:
            self.say("System paused")
        else:
            self.say("System active")

    def enable(self, name, service, state, check):
        enabled = None
        for x in xrange(2):
            try:
                rospy.loginfo("Enabling %s %s", name, state)
                srv_resp=service(state)
                enabled=srv_resp.state
                break
            except rospy.ServiceException, e:
                rospy.logerr("Unable to use %s enable service: %s", name, e)
                if check(state):
                    enabled = state
                    break
                else:
                    try:
                        service(False)
                    except rospy.ServiceException, e:
                        pass
        return enabled

    def pause(self, state):
        rospy.logdebug("pause %s", state)
        if self.check_wheelpod_status(not state) and \
                self.carousel_servo_status == (not state):
            self.set_pause_state(state)
            return
        wheelpods_paused = not self.enable("wheelpods",
                                           self.enable_wheelpods,
                                           not state,
                                           self.check_wheelpod_status)
        carousel_paused = not self.enable("carousel",
                                          self.enable_carousel,
                                          not state,
                                          lambda x:x==self.carousel_servo_status)
        if wheelpods_paused == state and carousel_paused == state:
            self.set_pause_state(state)
            self.announce_pause_state()
        elif wheelpods_paused == state:
            self.say("Wheelpods %s, carousel failed"%{True:"paused",False:"active"}[state])
        elif carousel_paused == state:
            self.say("Carousel %s, wheel pods failed"%{True:"paused",False:"active"}[state])
        else:
            self.say("Pause switch failed")

if __name__=="__main__":
    rospy.init_node('pause_switch')
    p=PauseSwitch()
    rospy.spin()
