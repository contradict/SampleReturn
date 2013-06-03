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

        rospy.loginfo('Pause_switch waiting for servo controller enable service...')
        rospy.wait_for_service('/enable_wheel_pods')
        rospy.wait_for_service('/enable_carousel')
        self.enable_wheelpods = rospy.ServiceProxy('/enable_wheel_pods', Enable)
        self.enable_carousel = rospy.ServiceProxy('/enable_carousel', Enable)

        rospy.loginfo('Pause_switch waiting for manipulator/pause service...')
        rospy.wait_for_service('/manipulator/pause')
        self.manipulator_pause_service = rospy.ServiceProxy('/manipulator/pause', Enable)

        rospy.Subscriber("/gpio_read", GPIO, self.gpio)
        rospy.Subscriber("/status_word", ServoStatus, self.status_word)

        self.pause_pub = rospy.Publisher("pause_state", Bool)
        self.audio_pub = rospy.Publisher("/audio/search", SoundRequest)

        #pullup on pause input, 1->0 is transition edge
        self.pause_bit_state = self.button_mask 
        self.guarded = False #this flag is set to true after a pause
        self.paused = True
        self.pause(True) #pause system on startup, no matter what

    def clear_guard(self, event):
        self.guarded = False

    def gpio(self, gpio):
        if gpio.servo_id == self.gpio_servo_id:
            new_bit_state = gpio.new_pin_states & self.button_mask
            if (self.pause_bit_state == self.button_mask and new_bit_state == 0):
                new_paused_state = None
                if self.paused and not self.guarded:
                    new_paused_state = False #unpause on transition if previously
                else:                        #paused and guard period over
                    new_paused_state = True #allow repausing immediately
                if new_paused_state is not None: self.pause(new_paused_state)
            self.pause_bit_state = new_bit_state

    def status_word(self, status_word):
        if status_word.servo_id == self.carousel_servo_id:
            self.carousel_servo_status = "Operation Enabled" in status_word.status
        if status_word.servo_id in self.wheelpod_servo_ids:
            self.wheelpod_servo_status[status_word.servo_id] = \
                ("Operation Enabled" in status_word.status) and \
                ("Switch On Disabled" not in status_word.status)
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
        if self.paused:
            self.guarded = True
            rospy.Timer(rospy.Duration(3.0), self.clear_guard, oneshot = True)
 
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
        wheelpods_toggled = not state == self.enable("wheelpods",
                                                     self.enable_wheelpods,
                                                     not state,
                                                     self.check_wheelpod_status)
        carousel_toggled = not state == self.enable("carousel",
                                                    self.enable_carousel,
                                                    not state,
                                                    lambda x:x==self.carousel_servo_status)
        manipulator_toggled = self.manipulator_pause_service(state).state == state
        if wheelpods_toggled and carousel_toggled and manipulator_toggled:
            self.set_pause_state(state)
            self.announce_pause_state()
        else:
            state_str = 'paused' if state else 'active'
            message =  "Wheelpods %s, "%(state_str if wheelpods_toggled else "failed")
            message += "Carousel %s, "%(state_str if carousel_toggled else "failed")
            message += "Manipulator %s. "%(state_str if manipulator_toggled else "failed")
            self.say(message)
       

if __name__=="__main__":
    rospy.init_node('pause_switch')
    p=PauseSwitch()
    rospy.spin()
