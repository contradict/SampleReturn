#!/usr/bin/env python
import sys
import math
import rospy
import rosnode
# still a rosbuild package, this is annoying
sys.path.append('/opt/ros/groovy/stacks/executive_teer/teer_ros/src')
import teer_ros
import actionlib
import actionlib_msgs.msg as action_msg
import tf

import std_msgs.msg as std_msgs
sys.path.append('/opt/ros/groovy/stacks/audio_common/sound_play/src/')
from sound_play.msg import SoundRequest
import sensor_msgs.msg as sensor_msgs
import platform_motion.msg as platform_msg
import platform_motion.srv as platform_srv
import manipulator.msg as manipulator_msg
import geometry_msgs.msg as geometry_msg
import linemod_detector.msg as detector_msg
import movebase_msg.msg as move_base_msg

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
    current_nav_sample = teer_ros.ConditionVariable(None)
    current_man_sample = teer_ros.ConditionVariable(None)

    proclamations = teer_ros.ConditionVariable([])

    working_states = [action_msg.GoalStatus.ACTIVE, action_msg.GoalStatus.PENDING]

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
        self.precached_sample_distance = rospy.get_param("precached_sample_distance", 10.0)

        # subscribe to interesting topics
        rospy.Subscriber("/gpio_read", platform_msg.GPIO, self.gpio_update)
        rospy.Subscriber("/navigation/camera_status", std_msgs.String,
                self.navigation_status_update)
        rospy.Subscriber("/manipulator/camera_status", std_msgs.String,
                self.manipulator_status_update)
        rospy.Subscriber("/pause_state", std_msgs.Bool,
                self.pause_state_update)
        rospy.Subscriber("/navigation/sample_detections", detector_msg.NamedPoint,
                self.sample_detection_nav_update)
        rospy.Subscriber("/manipulator/sample_detections", detector_msg.NamedPoint,
                self.sample_detection_man_update)

        self.transformer = tf.Transformer(False, rospy.Duration(10.0))

        # create publishers
        self.navigation_audio=rospy.Publisher("/audio/navigate", SoundRequest)

        # create serivce proxies
        self.platform_motion_input_select = \
                rospy.ServiceProxy("/select_command_source",
                        platform_srv.SelectCommandSource)

        # action clients
        self.home_wheelpods = actionlib.SimpleActionClient("/home_wheelpods",
                                                           platform_msg.HomeAction)
        self.home_carousel = actionlib.SimpleActionClient("/home_carousel",
                                                          platform_msg.HomeAction)
        self.home_manipulator = actionlib.SimpleActionClient('/manipulator/manipulator_action',
                                                             manipulator_msg.ManipulatorAction)

        self.move_base = actionlib.SimpleActionClient("/planner/move_base",
                move_base_msg.MoveBaseAction)
        self.servo = actionlib.SimpleActionClient("/servo/acquire",
                visual_servo_msg.AcquireAction)

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

    def sample_detection_nav_update(self, msg):
        self.current_nav_sample = msg

    def sample_detection_man_update(self, msg):
        self.current_man_sample = msg

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

        while True: #this loop waits for a full, successful homing of the system
            if not enabled():
                self.announce("Waiting for system enable.")
                yield teer_ros.WaitCondition(enabled)
            self.announce("Home ing")
            
            mh_msg = manipulator_msg.ManipulatorGoal()
            mh_msg.type = 'home'
            self.home_manipulator.send_goal(mh_msg)
            while True: #home manipulator first, ensure it is clear of carousel
                yield teer_ros.WaitDuration(0.1)
                manipulator_state = self.home_manipulator.get_state()
                if manipulator_state not in self.working_states:
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
                if wheelpods_state not in self.working_states \
                   and carousel_state not in self.working_states:
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
                self.retrieve_precached_sample()
            else:
                self.announce("Joystick control enabled")
                self.platform_motion_input_select("Joystick")
            pin_states =\
            self.gpio.new_pin_states&(self.GPIO_PIN_MODE_PRECACHED|self.GPIO_PIN_MODE_SEARCH)
            yield teer_ros.WaitCondition(
                    lambda: self.gpio.new_pin_states&(self.GPIO_PIN_MODE_PRECACHED|self.GPIO_PIN_MODE_SEARCH) != pin_states)


    def servo_feedback_cb(self, feedback):
        if len(self.proclamations) == 0:
            self.announce("distance to sample %3.1 meters"%feedback.distance)

    def retrieve_precached_sample(self):
        self.current_nav_sample = None
        self.current_man_sample = None
        self.platform_motion_input_select("Planner")
        expected_position = move_base_msg.MoveBaseGoal()
        expected_position.target_pose.header.stamp=rospy.Time.now()
        expected_position.target_pose.header.frame_id='/map'
        expected_position.target_pose.pose.position.x=self.precached_sample_distance
        expected_position.target_pose.pose.position.y=0
        expected_position.target_pose.pose.position.z=0
        expected_position.target_pose.pose.orientation= \
          geometry_msg.Quaterion(*tf.transformations.quaternion_from_euler(0,0,0))
        self.announce("Moving to expected sample position")
        self.move_base.send_goal(expected_position)
        yield teer_ros.WaitCondition(
                lambda: self.current_nav_sample is not None and \
                self.current_nav_sample.name=="Pre-cached Sample")
        self.move_base.cancel_goal()
        self.announce("%s detected, resetting goal"%self.current_nav_sample.name)
        target_frame = self.current_nav_sample.header.frame
        trans, quat = tf.lookupTransform('/map', target_frame, rospy.Time())
        actual_position = expected_position
        actual_position.target_pose.header.stamp=rospy.Time.now()
        actual_position.pose.position.x = self.current_nav_sample.point.x+trans[0]
        actual_position.pose.position.y = self.current_nav_sample.point.y+trans[0]
        self.mover_base.send_goal(actual_position)
        yield teer_ros.WaitCondition(
                lambda: self.current_man_sample is not None and \
                        self.current_nav_sample.name=="Pre-cached Sample")
        self.move_base.cancel_goal()
        self.platform_motion_input_select("Servo")
        self.announce("%s in manipulator view, switching to servo control"%self.current_man_sample)
        self.servo.send_goal(visual_servo_msg.AcquireGoal(),
                             feedback_cb=self.servo_feedback_cb
                            )
        while True:
            yield teer_ros.WaitDuration(0.1)
            state = self.servo.get_state()
            if state not in self.working_states:
                break
        self.announce("Servo success, triggering manipulator sequence")

        self.manipulator.send_goal(manipulator_msg.ManipulatorGrabGoal())
        while True:
            yield teer_ros.WaitDuration(0.1)
            state = self.manipulator.get_state()
            if state not in self.working_states:
                break

        self.announce("Sample retreived, returning to starting point")
        home_goal = move_base_msg.MoveBaseGoal()
        home_goal.target_pose.header.stamp=rospy.Time.now()
        home_goal.target_pose.header.frame_id='/map'
        home_goal.target_pose.pose.position.x=0
        home_goal.target_pose.pose.position.y=0
        home_goal.target_pose.pose.position.z=0
        home_goal.target_pose.pose.orientation= \
          geometry_msg.Quaterion(*tf.transformations.quaternion_from_euler(0,0,math.pi))
        self.move_base.send_goal(home_goal)
        while True:
            yield teer_ros.WaitDuration(0.1)
            state = self.move_base.get_state()
            if state not in self.working_states:
                break
        self.announce("Complete, waiting for mode change")
        mask = (self.GPIO_PIN_MODE_PRECACHED|self.GPIO_PIN_MODE_SEARCH)
        pin_states =\
          self.gpio.new_pin_states&mask
        yield teer_ros.WaitCondition(
                lambda: self.gpio.new_pin_states&mask != pin_states)


def start_node():
    rospy.init_node('master_executive')
    sched = SampleReturnScheduler()
    # kick off startup task
    sched.new_task(sched.announcer())
    sched.new_task(sched.start_robot())
    sched.run()

if __name__=="__main__":
    start_node()
