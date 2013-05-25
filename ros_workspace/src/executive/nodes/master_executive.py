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

import std_msgs.msg as std_msg
sys.path.append('/opt/ros/groovy/stacks/audio_common/sound_play/src/')
from sound_play.msg import SoundRequest
import sensor_msgs.msg as sensor_msgs
import platform_motion.msg as platform_msg
import platform_motion.srv as platform_srv
import manipulator.msg as manipulator_msg
import geometry_msgs.msg as geometry_msg
import move_base_msgs.msg as move_base_msg
import visual_servo.msg as visual_servo_msg
import linemod_detector.msg as linemod_msg

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
    man_sample = teer_ros.ConditionVariable(None)
    search_sample = teer_ros.ConditionVariable(None)
    beacon_vector = teer_ros.ConditionVariable(None)

    proclamations = teer_ros.ConditionVariable([])

    working_states = [action_msg.GoalStatus.ACTIVE, action_msg.GoalStatus.PENDING]

    def __init__(self):
        super(SampleReturnScheduler,self).__init__()

        # ordinary variables
        self.last_servo_feedback = None

        # node parameters
        self.voice = rospy.get_param("~voice", "kal_diphone")
        self.speech_delay = rospy.get_param("~speech_delay", 0.8)
        self.speech_rate = rospy.get_param("~speech_rate", 0.07)
        self.precached_sample_distance = rospy.get_param("~precached_sample_distance", 2.0)
        rospy.loginfo("distance: %f", self.precached_sample_distance)
        self.servo_feedback_interval =\
            rospy.Duration(rospy.get_param("~servo_feedback_interval", 5.0))
        self.gpio_servo_id = rospy.get_param("~gpio_servo_id", 1)
        self.pursuit_complete_distance = rospy.get_param("~pursuit_complete_distance", 5.0)
        self.home_pursuit_complete_distance = rospy.get_param("~home_pursuit_complete_distance", 10.0)
        self.maximum_pursuit_error = rospy.get_param("~maximum_pursuit_error", 1.5)

        # subscribe to interesting topics
        rospy.Subscriber("/gpio_read", platform_msg.GPIO, self.gpio_update)
        rospy.Subscriber("/navigation/camera_status", std_msg.String,
                self.navigation_status_update)
        rospy.Subscriber("/manipulator/camera_status", std_msg.String,
                self.manipulator_status_update)
        rospy.Subscriber("/pause_state", std_msg.Bool,
                self.pause_state_update)
        rospy.Subscriber("/sample_detection/filtered_points",
                linemod_msg.NamedPoint,
                self.sample_detection_search_update)
        rospy.Subscriber("/img_point", linemod_msg.NamedPoint,
                self.sample_detection_man_update)
        rospy.Subscriber("/sample_distance", std_msg.Float64,
                self.sample_distance_update)
        rospy.Subscriber("/beacon_vector", geometry_msg.Vector3,
                self.beacon_update)

        self.listener = tf.TransformListener()

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
        self.manipulator = actionlib.SimpleActionClient('/manipulator/manipulator_action',
                                                        manipulator_msg.ManipulatorAction)

        self.move_base = actionlib.SimpleActionClient("/planner/move_base",
                move_base_msg.MoveBaseAction)
        self.servo = actionlib.SimpleActionClient("/visual_servo_action",
                visual_servo_msg.VisualServoAction)

    #----   Subscription Handlers ----
    def gpio_update(self, gpio):
        if gpio.servo_id == self.gpio_servo_id:
            if self.gpio is not None and \
                    gpio.new_pin_states != self.gpio.new_pin_states:
                rospy.loginfo("gpio: %s", gpio)
            self.gpio = gpio

    def navigation_status_update(self, status):
        if self.navigation_camera_status != status:
            self.navigation_camera_status = status

    def manipulator_status_update(self, status):
        if self.manipulator_camera_status != status:
            self.manipulator_camera_status = status

    def sample_detection_search_update(self, msg):
        self.search_sample = msg

    def sample_detection_man_update(self, msg):
        self.man_sample = msg

    def beacon_update(self, msg):
        rospy.loginfo("Beacon vector: %s", msg)
        self.beacon_vector = msg

    def pause_state_update(self, state):
        self.pause_state = state
        rospy.logdebug("Pause state %s", state)

    def sample_distance_update(self, msg):
        self.precached_sample_distance = msg.data
        rospy.loginfo("Set sample distance to %f",
                self.precached_sample_distance)

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
            msg = self.proclamations[0]
            self.navigation_audio.publish(msg)
            rospy.logwarn("executive announce: %s", msg.arg)
            duration = self.speech_delay + self.speech_rate*len(msg.arg)
            yield teer_ros.WaitDuration(duration)
            self.proclamations.pop(0)

    #----   Tasks   ----
    def start_robot(self):
        yield teer_ros.WaitDuration(2.0)
        camera_ready = lambda: self.navigation_camera_status is not None and \
                        self.navigation_camera_status.data=="Ready" and \
                        self.manipulator_camera_status is not None and \
                        self.manipulator_camera_status.data=="Ready"
        if not camera_ready():
            self.announce("Waiting for cameras")
            yield teer_ros.WaitCondition(camera_ready)
        enabled = lambda: self.pause_state is not None and \
                          not self.pause_state.data

        while True:
            if self.home_wheelpods.wait_for_server(rospy.Duration(1e-6))\
            and self.home_carousel.wait_for_server(rospy.Duration(1e-6))\
            and self.manipulator.wait_for_server(rospy.Duration(1e-6)):
                break
            yield teer_ros.WaitDuration(0.1)

        while True: #this loop waits for a full, successful homing of the system
            if not enabled():
                self.announce("Waiting for system enable.")
                yield teer_ros.WaitCondition(enabled)
            self.announce("Home ing")

            mh_msg = manipulator_msg.ManipulatorGoal()
            mh_msg.type = mh_msg.HOME
            self.manipulator.send_goal(mh_msg)
            while True: #home manipulator first, ensure it is clear of carousel
                yield teer_ros.WaitDuration(0.1)
                manipulator_state = self.manipulator.get_state()
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
            tid=None
            rospy.logdebug("pins: %s", hex(self.gpio.new_pin_states))
            if self.gpio.new_pin_states&self.GPIO_PIN_MODE_SEARCH == 0:
                self.announce("Entering search mode")
                self.platform_motion_input_select("Planner")
            elif self.gpio.new_pin_states&self.GPIO_PIN_MODE_PRECACHED == 0:
                self.announce("Entering pree cashed sample mode")
                tid = self.new_task(self.retrieve_precached_sample())
            else:
                self.announce("Joystick control enabled")
                self.platform_motion_input_select("Joystick")
            pin_states =\
            self.gpio.new_pin_states&(self.GPIO_PIN_MODE_PRECACHED|self.GPIO_PIN_MODE_SEARCH)
            yield teer_ros.WaitCondition(
                    lambda: self.gpio.new_pin_states&(self.GPIO_PIN_MODE_PRECACHED|self.GPIO_PIN_MODE_SEARCH) != pin_states)
            if tid is not None:
                try:
                    self.kill_task(tid)
                except:
                    pass

    def servo_feedback_cb(self, feedback):
        now = rospy.Time.now()
        if self.last_servo_feedback is None or \
            (len(self.proclamations) == 0 and\
              now-self.last_servo_feedback>self.servo_feedback_interval):
            if feedback.state == feedback.STOP_AND_WAIT:
                self.announce("lost sample")
            else:
                self.announce("range %d"%(10*int(feedback.error/10)))
            self.last_servo_feedback = now

    def get_current_robot_pose(self):
        self.listener.waitForTransform('/map', '/base_link',
                rospy.Time(0), rospy.Duration(10.0))
        position, quaternion = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        hdr = std_msg.Header(0, rospy.Time.now(), '/map')
        pose = geometry_msg.PoseStamped(hdr,
                geometry_msg.Pose(
                    geometry_msg.Point(*position),
                    geometry_msg.Quaternion(*quaternion)))
        return pose

    def expected(self, start_pose):
        rospy.loginfo('start pose: %s', start_pose)
        hdr = std_msg.Header(0, rospy.Time(0), '/base_link')
        ahead=geometry_msg.PointStamped(hdr,
                geometry_msg.Point(self.precached_sample_distance, 0, 0))
        desired_pt=self.listener.transformPoint('/map', ahead)
        rospy.loginfo("desired_pt: %s", desired_pt)
        # at desired_pt
        # with current orientation
        expected_pose = \
            geometry_msg.Pose(desired_pt.point, start_pose.pose.orientation)
        hdr =std_msg.Header(0, rospy.Time(0), '/map')
        pose_st = geometry_msg.PoseStamped(hdr, expected_pose)
        rospy.loginfo('expected pose: %s', pose_st)
        return pose_st

    def spin(self, current_pose, start_pose):
        # spin in place to point at home
        around = tf.transformations.quaternion_from_euler(0,0,math.pi)
        q1 = (start_pose.orientation.x, start_pose.orientation.y,
                start_pose.orientation.z, start_pose.orientation.w)
        spin_orientation = \
                geometry_msg.Quaternion(*tf.transformations.quaternion_multiply(around, q1))
        spin_pose = geometry_msg.Pose(current_pose.position,
                spin_orientation)
        hdr = std_msg.Header(0, rospy.Time(0), '/map')
        spin_goal = move_base_msg.MoveBaseGoal()
        spin_goal.target_pose = \
                geometry_msg.PoseStamped(hdr, spin_pose)
        rospy.loginfo("spin goal: %s", spin_goal)
        self.move_base.send_goal(spin_goal)
        return spin_pose

    def home(self, start_pose, spin_pose):
        # drive back to (0,0,0)
        hdr = std_msg.Header(0, rospy.Time(0), '/map')
        home_pose = geometry_msg.Pose(start_pose.position,
                        spin_pose.orientation)
        target_pose=\
            geometry_msg.PoseStamped(hdr, home_pose)
        rospy.loginfo("home goal: %s", target_pose)
        return target_pose

    def drive_to_point(self, pose_st):
        goal = move_base_msg.MoveBaseGoal()
        goal.target_pose=pose_st
        self.move_base.send_goal(goal)
        while True:
            yield teer_ros.WaitDuration(0.1)
            move_state = self.move_base.get_state()
            if move_state not in self.working_states:
                break

    def wait_for_search_sample(self):
        # set sample to None so we know when the first point arrives
        self.search_sample = None
        # wait for the search camera to see the object
        yield teer_ros.WaitCondition(
                lambda: self.search_sample is not None)
        self.announce("Sample located")

    def wait_for_manipulator_sample(self):
        # set sample to None so we know when the first point arrives
        self.manipulator_sample = None
        # wait for the manipulator camera to see the object
        yield teer_ros.WaitCondition(
                lambda: self.search_sample is not None)
        self.announce("Sample located")
        # wait to stop
        self.move_base.cancel_goal()
        yield teer_ros.WaitDuration(0.5)

    def point_distance(self, pt1, pt2):
        return math.sqrt((pt1.x-pt2.x)**2 + (pt1.y-pt2.y)**2)

    def pursue(self, pose_st):
        goal = move_base_msg.MoveBaseGoal()
        goal.target_pose=pose_st
        self.move_base.send_goal(goal)

        while True:
            yield teer_ros.WaitDuration(0.5)
            move_state = self.move_base.get_state()
            if move_state not in self.working_states:
                break
            robot_pose_st = self.get_current_robot_pose()
            if self.point_distance(pose_st.pose.position,
                    robot_pose_st.pose.position)<self.pursuit_complete_distance:
                continue
            if self.search_sample is not None:
                msg = self.search_sample
                hdr = std_msg.Header(0, rospy.Time(0), 'search_camera_lens')
                ps = geometry_msg.PointStamped(hdr, msg.pose.position)
                newpt = self.listener.transformPoint('/map', ps).pose.position
                current_point = pose_st.pose.position
                if self.point_distance(mappt, newpt) > self.maximum_pursuit_error:
                    pose_st.pose.position=newpt
                    goal.target_pose=pose_st
                    self.move_base.send_goal(goal)
                self.search_sample = None

    def drive_to_sample(self, start_pose):
        # switch control to the motion planner and drive to the expected
        # position
        self.announce("Moving to expected sample position")
        self.platform_motion_input_select("Planner")
        expected_position = self.expected(start_pose)
        drive=self.new_task(self.drive_to_point(expected_position))
        find=self.new_task(self.wait_for_search_sample())
        yield teer_ros.WaitAnyTasks([drive, find])
        move_state = self.move_base.get_state()
        if move_state == action_msg.GoalStatus.SUCCEEDED:
            # arrived at goal position without seeing sample
            self.announce("Failed to see sample, returning")
            self.kill_task(find)
            self.new_task(self.drive_home(start_pose))
        else:
            # see sample in search, update goal and drive close
            msg = self.search_sample
            self.listener.waitForTransform('/map', msg.header.frame_id,
                    rospy.Time(0), rospy.Duration(10.0))
            mappt = self.listener.transformPoint('/map', msg)
            expected_position.pose.position=mappt
            pursue = self.new_task(self.pursue(expected_position))
            see = self.new_task(self.wait_for_manipulator_sample)
            yield teer_ros.WaitAnyTasks([pursue, see])
            if self.man_sample is None:
                self.announce("Failed to acquire sample in manipulator camera")
                self.new_task(self.drive_home(start_pose))
            self.drive_succeeded=True

    def acquire_sample(self):
        # switch to visual servo control
        self.announce("Sample detected, servoing")
        self.platform_motion_input_select("Servo")
        self.servo.send_goal(visual_servo_msg.VisualServoGoal(),
                             feedback_cb=self.servo_feedback_cb
                            )
        while True:
            yield teer_ros.WaitDuration(0.1)
            state = self.servo.get_state()
            if state not in self.working_states:
                break
        self.announce("Servo success, triggering manipulator")

        # grab the sample
        manip_goal = manipulator_msg.ManipulatorGoal()
        manip_goal.type=manip_goal.GRAB
        manip_goal.target_bin = 1
        manip_goal.grip_torque = 0.5
        self.manipulator.send_goal(manip_goal)
        while True:
            yield teer_ros.WaitDuration(0.1)
            state = self.manipulator.get_state()
            if state not in self.working_states:
                break
        self.announce("Sample retreived")

    def beacon_goal(self, current_beacon_point, new_beacon_vector, robot_pose):
        hdr = std_msg.Header(0, rospy.Time(0), 'search_camera_lens')
        ps=geometry_msg.PointStamped(hdr, new_beacon_vector)
        newpt = self.listener.transformPoint('/map', ps).point
        rospy.loginfo("newpt: %s", newpt)
        if self.point_distance(current_beacon_point.pose.position, newpt) > self.maximum_pursuit_error:
            self.announce("recentering beacon")
            current_beacon_point.pose.position = newpt
            g_position = geometry_msg.Point(newpt.x+1.0, newpt.y, newpt.z)
            g_pose = geometry_msg.Pose(g_position, current_beacon_point.pose.orientation)
            g=geometry_msg.PoseStamped(current_beacon_point.header, g_pose)
            rospy.loginfo("g: %s", g)
            return g
        return None


    def drive_home(self, start_pose):
        # spin in place to point at home
        self.platform_motion_input_select("Planner")
        capture_pose = self.get_current_robot_pose()
        spin_pose = self.spin(capture_pose.pose, start_pose.pose)
        while True:
            yield teer_ros.WaitDuration(0.1)
            state = self.move_base.get_state()
            if state not in self.working_states:
                break
        rospy.loginfo("spin complete")
        # drive back to beacon
        home_goal = move_base_msg.MoveBaseGoal()
        home_goal.target_pose = self.home(start_pose.pose, spin_pose)
        self.move_base.send_goal(home_goal)
        current_beacon_point = home_goal.target_pose
        self.beacon_vector = None
        while True:
            yield teer_ros.WaitDuration(0.5)
            move_state = self.move_base.get_state()
            if move_state not in self.working_states:
                rospy.loginfo("move done")
                break
            robot_pose_st = self.get_current_robot_pose()
            dist = self.point_distance(home_goal.target_pose.pose.position, robot_pose_st.pose.position)
            if dist<self.home_pursuit_complete_distance:
                rospy.loginfo("close enough: %f", dist)
                continue
            if self.beacon_vector is not None:
                rospy.loginfo("new beacon")
                g_p_st = self.beacon_goal(current_beacon_point, self.beacon_vector, robot_pose_st)
                if g_p_st is not None:
                    home_goal.target_pose=g_p_st
                    self.move_base.send_goal(home_goal)
                else:
                    rospy.loginfo("current goal close enough")
                self.beacon_vector = None


        self.announce("Complete, waiting for mode change")
        mask = (self.GPIO_PIN_MODE_PRECACHED|self.GPIO_PIN_MODE_SEARCH)
        pin_states =\
          self.gpio.new_pin_states&mask
        yield teer_ros.WaitCondition(
                lambda: self.gpio.new_pin_states&mask != pin_states)
        rospy.loginfo("mode changed")


    def retrieve_precached_sample(self):
        # write down sart pose
        start_pose = self.get_current_robot_pose()
        rospy.loginfo('start pose: %s', start_pose)

        self.drive_succeeded=False
        yield teer_ros.WaitTask(self.new_task(self.drive_to_sample(start_pose)))
        if not self.drive_succeeded:
            return
        yield teer_ros.WaitTask(self.new_task(self.acquire_sample()))

        yield teer_ros.WaitTask(self.new_task(self.drive_home(start_pose)))

def start_node():
    rospy.init_node('master_executive')
    sched = SampleReturnScheduler()
    # kick off startup task
    sched.new_task(sched.announcer())
    sched.new_task(sched.start_robot())
    sched.run()

if __name__=="__main__":
    start_node()
