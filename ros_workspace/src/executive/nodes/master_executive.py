#!/usr/bin/env python
import sys
import rospy
import rosnode
# still a rosbuild package, this is annoying
sys.path.append('/opt/ros/groovy/stacks/executive_teer/teer_ros/src')
import teer_ros
import actionlib
import actionlib_msgs.msg as action_msg

import std_msgs.msg as std_msgs
sys.path.append('/opt/ros/groovy/stacks/audio_common/sound_play/src/')
from sound_play.msg import SoundRequest
import sensor_msgs.msg as sensor_msgs
import platform_motion.msg as platform_msg
import platform_motion.srv as platform_srv

class SampleReturnScheduler(teer_ros.Scheduler):
    GPIO_PIN_PAUSE=0x02
    GPIO_PIN_BATTERY_STATE=0x04
    GPIO_PIN_MODE_SEARCH=0x08
    GPIO_PIN_MODE_PRECACHED=0x20

    # condition variables
    # These are how teer waits for state changes
    gpio = teer_ros.ConditionVariable(None)
    navigation_camera_status = teer_ros.ConditionVariable(None)
    manipulator_camera_status = teer_ros.ConditionVariable(None)
    pause_state = teer_ros.ConditionVariable(None)
    proclamations = teer_ros.ConditionVariable([])

    def __init__(self):
        super(SampleReturnScheduler,self).__init__()

        # ordinary variables

        # node parameters
        self.voice = rospy.get_param("voice", "kal_diphone")
        self.maximum_desync = rospy.get_param("maximum_desync", 0.10)
        self.desync_wait_count = rospy.get_param("desync_wait_count", 5)
        self.resync_wait = rospy.get_param("resync_wait", 10)
        self.speech_delay = rospy.get_param("speech_delay", 0.8)
        self.speech_rate = rospy.get_param("speech_rate", 0.07)

        # subscribe to interesting topics
        rospy.Subscriber("/gpio_read", platform_msg.GPIO, self.gpio_update)
        rospy.Subscriber("/navigation/camera_status", std_msgs.String,
                self.navigation_status_update)
        rospy.Subscriber("/manipulator/camera_status", std_msgs.String,
                self.manipulator_status_update)
        rospy.Subscriber("/pause_state", std_msgs.Bool,
                self.pause_state_update)

        # create publishers
        self.navigation_audio=rospy.Publisher("/audio/navigate", SoundRequest)

        # create serivce proxies
        self.platform_motion_input_select = \
                rospy.ServiceProxy("/select_command_source",
                        platform_srv.SelectCommandSource)
        self.enable_wheelpods = rospy.ServiceProxy('enable_wheel_pods',
                platform_srv.Enable)
        self.enable_carousel = rospy.ServiceProxy('enable_carousel',
                platform_srv.Enable)

        # action clients
        self.home_wheelpods = actionlib.SimpleActionClient("/home_wheelpods",
                                                           platform_msg.HomeAction)
        self.home_carousel = actionlib.SimpleActionClient("/home_carousel",
                                                          platform_msg.HomeAction)
        self.home_manipulator = actionlib.SimpleActionClient('/manipulator/home',
                                                             platform_msg.HomeAction)

    #----   Subscription Handlers ----
    def gpio_update(self, gpio):
        self.gpio = gpio

    def navigation_status_update(self, status):
        if self.navigation_camera_status != status:
            self.navigation_camera_status = status

    def manipulator_status_update(self, status):
        if self.manipulator_camera_status != status:
            self.manipulator_camera_status = status

    def pause_state_update(self, state):
        self.pause_state = state
        rospy.logdebug("Pause state %s", state)

    #----   Publisher Helpers ----
    def announce(self, utterance):
        msg = SoundRequest()
        msg.sound = SoundRequest.SAY
        msg.command = SoundRequest.PLAY_ONCE
        msg.arg = utterance
        msg.arg2 = 'voice.select "%s"'%self.voice
        self.proclamations.append(msg)

    def announcer(self):
        while True:
            if(len(self.proclamations)==0):
                yield teer_ros.WaitDuration(0.1)
                continue
            #yield teer_ros.WaitCondition(
            #        lambda: len(self.proclamations)>0)
            msg = self.proclamations.pop(0)
            self.navigation_audio.publish(msg)
            rospy.logwarn("executive announce: %s", msg.arg)
            duration = self.speech_delay + self.speech_rate*len(msg.arg)
            yield teer_ros.WaitDuration(duration)

    #----   Tasks   ----
    def start_robot(self):
        yield teer_ros.WaitDuration(2.0)
        camera_ready = lambda: self.navigation_camera_status is not None and \
                        self.navigation_camera_status.data=="Ready" #and \
                        #self.manipulator_camera_status is not None and \
                        #self.manipulator_camera_status.data=="Ready"
        if not camera_ready():
            self.announce("Waiting for cameras")
            yield teer_ros.WaitCondition(camera_ready)
        enabled = lambda: self.pause_state is not None and \
                          not self.pause_state.data

        while True:
            if self.home_wheelpods.wait_for_server(rospy.Duration(1e-6))\
            and self.home_carousel.wait_for_server(rospy.Duration(1e-6))\
            and self.home_manipulator.wait_for_server(rospy.Duration(1e-6)):
                break
            yield teer_ros.WaitDuration(0.1)

        while True:
            if not enabled():
                self.announce("Waiting for system enable.")
                yield teer_ros.WaitCondition(enabled)
            self.announce("Home ing")
            
            working_states = [action_msg.GoalStatus.ACTIVE, action_msg.GoalStatus.PENDING]

            self.home_manipulator.send_goal(platform_msg.HomeGoal(home_count=1))
            while True: #home manipulator first, ensure it is clear of carousel
                yield teer_ros.WaitDuration(0.1)
                manipulator_state = self.home_manipulator.get_state()
                if manipulator_state not in working_states:
                    break
            if manipulator_state != action_msg.GoalStatus.SUCCEEDED:
                yield teer_ros.WaitDuration(0.75)
                continue
            
            self.platform_motion_input_select("None")
            yield teer_ros.WaitDuration(0.1)
            self.home_wheelpods.send_goal(platform_msg.HomeGoal(home_count=3))
            self.home_carousel.send_goal(platform_msg.HomeGoal(home_count=1))
            while True:
                yield teer_ros.WaitDuration(0.10)
                wheelpods_state = self.home_wheelpods.get_state()
                carousel_state = self.home_carousel.get_state()
                if wheelpods_state not in working_states \
                   and carousel_state not in working_states:
                    break
            rospy.logdebug("home results: (%d, %d)", wheelpods_state, carousel_state)
            if wheelpods_state == action_msg.GoalStatus.SUCCEEDED \
               and carousel_state == action_msg.GoalStatus.SUCCEEDED:
                break
            yield teer_ros.WaitDuration(0.75)
        self.new_task(self.handle_mode_switch())

    def handle_mode_switch(self):
        yield teer_ros.WaitCondition(lambda: self.gpio is not None)

        while True:
            rospy.logdebug("pins: %s", hex(self.gpio.new_pin_states))
            if self.gpio.new_pin_states&self.GPIO_PIN_MODE_SEARCH == 0:
                self.announce("Entering search mode")
                self.platform_motion_input_select("Planner")
            elif self.gpio.new_pin_states&self.GPIO_PIN_MODE_PRECACHED == 0:
                self.announce("Entering pree cashed sample mode")
                self.platform_motion_input_select("Planner")
            else:
                self.announce("Joystick control enabled")
                self.platform_motion_input_select("Joystick")
            pin_states =\
            self.gpio.new_pin_states&(self.GPIO_PIN_MODE_PRECACHED|self.GPIO_PIN_MODE_SEARCH)
            yield teer_ros.WaitCondition(
                    lambda: self.gpio.new_pin_states&(self.GPIO_PIN_MODE_PRECACHED|self.GPIO_PIN_MODE_SEARCH) != pin_states)



def start_node():
    rospy.init_node('master_executive')
    sched = SampleReturnScheduler()
    # kick off startup task
    sched.new_task(sched.announcer())
    sched.new_task(sched.start_robot())
    sched.run()

if __name__=="__main__":
    start_node()
