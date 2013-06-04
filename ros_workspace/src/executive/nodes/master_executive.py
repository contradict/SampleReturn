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
import tf_conversions

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
    beacon_pose = teer_ros.ConditionVariable(None)

    proclamations = teer_ros.ConditionVariable([])

    working_states = [action_msg.GoalStatus.ACTIVE, action_msg.GoalStatus.PENDING]

    def __init__(self):
        super(SampleReturnScheduler,self).__init__()

        # ordinary variables
        self.last_servo_feedback = None
        self.lost_sample = None

        # node parameters
        self.voice = rospy.get_param("~voice", "kal_diphone")
        self.speech_delay = rospy.get_param("~speech_delay", 0.8)
        self.speech_rate = rospy.get_param("~speech_rate", 0.07)
        self.servo_feedback_interval =\
            rospy.Duration(rospy.get_param("~servo_feedback_interval", 5.0))
        self.gpio_servo_id = rospy.get_param("~gpio_servo_id", 1)
        self.pursuit_complete_distance = rospy.get_param("~pursuit_complete_distance", 5.0)
        self.home_pursuit_complete_distance = rospy.get_param("~home_pursuit_complete_distance", 10.0)
        self.maximum_pursuit_error = rospy.get_param("~maximum_pursuit_error", 1.5)
        self.beacon_scan_distance = rospy.get_param("~beacon_scan_distance", 10.0)
        self.wait_for_cameras = rospy.get_param("~wait_for_cameras", True)

        # subscribe to interesting topics
        rospy.Subscriber("/gpio_read", platform_msg.GPIO, self.gpio_update)
        rospy.Subscriber("/navigation/camera_status", std_msg.String,
                self.navigation_status_update)
        rospy.Subscriber("/manipulator/camera_status", std_msg.String,
                self.manipulator_status_update)
        rospy.Subscriber("/pause_state", std_msg.Bool,
                self.pause_state_update)
        rospy.Subscriber("search_point",
                linemod_msg.NamedPoint,
                self.sample_detection_search_update)
        rospy.Subscriber("manipulator_point", linemod_msg.NamedPoint,
                self.sample_detection_man_update)
        rospy.Subscriber("/beacon_pose", geometry_msg.PoseStamped,
                self.beacon_update)

        self.listener = tf.TransformListener()

        # create publishers
        self.navigation_audio=rospy.Publisher("/audio/navigate", SoundRequest)
        self.joystick_command=rospy.Publisher("/joystick_command", geometry_msg.Twist)
        self.planner_command=rospy.Publisher("/planner_command", geometry_msg.Twist)
        self.servo_command=rospy.Publisher("/servo_command", geometry_msg.Twist)

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

        self.drive_home_task = None

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
        rospy.loginfo("Beacon Pose: %s", msg)
        self.beacon_pose = msg
        self.saw_beacon = True

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
            msg = self.proclamations[0]
            self.navigation_audio.publish(msg)
            rospy.logwarn("executive announce: %s", msg.arg)
            duration = self.speech_delay + self.speech_rate*len(msg.arg)
            yield teer_ros.WaitDuration(duration)
            self.proclamations.pop(0)

    def publish_zero_velocity(self):
        t=geometry_msg.Twist()
        t.linear.x=0
        t.linear.y=0
        t.linear.z=0
        t.angular.x=0
        t.angular.y=0
        t.angular.z=0
        self.joystick_command.publish(t)
        self.planner_command.publish(t)
        self.servo_command.publish(t)

    #----   Tasks   ----
    def start_robot(self):
        yield teer_ros.WaitDuration(2.0)
        if self.wait_for_cameras:
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
                if self.drive_home_task is not None:
                    self.kill_task(self.drive_home_task)
                    self.drive_home_task = None
                self.announce("Joystick control enabled")
                self.platform_motion_input_select("Joystick")
                self.publish_zero_velocity()
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
        if self.last_servo_feedback is None:
            self.last_servo_feedback = now
            return
        dt = (now-self.last_servo_feedback)
        if len(self.proclamations) == 0 and dt>self.servo_feedback_interval:
            if feedback.state == feedback.STOP_AND_WAIT:
                self.announce("lost sample")
            else:
                self.announce("range %d"%(10*int(feedback.error/10)))
            self.last_servo_feedback = now
            self.lost_sample = (feedback.state == feedback.STOP_AND_WAIT)

    def get_current_robot_pose(self):
        self.listener.waitForTransform('/map', '/base_link',
                rospy.Time(0), rospy.Duration(10.0))
        position, quaternion = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        hdr = std_msg.Header(0, rospy.Time(0), '/map')
        pose = geometry_msg.PoseStamped(hdr,
                geometry_msg.Pose(
                    geometry_msg.Point(*position),
                    geometry_msg.Quaternion(*quaternion)))
        return pose

    def forward(self, distance):
        start_pose = self.get_current_robot_pose()
        rospy.loginfo('start pose: %s', start_pose)
        hdr = std_msg.Header(0, rospy.Time(0), '/base_link')
        ahead=geometry_msg.PointStamped(hdr,
                geometry_msg.Point(distance, 0, 0))
        desired_pt = self.listener.transformPoint('/map', ahead)
        rospy.loginfo('desired point: %s', desired_pt)
        desired_pose = geometry_msg.Pose(desired_pt.point, start_pose.pose.orientation)
        hdr = std_msg.Header(0, rospy.Time(0), '/map')
        pose_st = geometry_msg.PoseStamped(hdr, desired_pose)
        rospy.loginfo('forward %f', distance)
        rospy.loginfo('desired pose: %s', pose_st)
        return pose_st

    def rotate(self, angle):
        current_pose = self.get_current_robot_pose()
        rospy.loginfo('current pose: %s', current_pose)
        goal_quat = tf.transformations.quaternion_from_euler(0,0,math.radians(angle))
        current_quat = (current_pose.pose.orientation.x, current_pose.pose.orientation.y,
                    current_pose.pose.orientation.z, current_pose.pose.orientation.w)
        spin_orientation = \
            geometry_msg.Quaternion(*tf.transformations.quaternion_multiply(goal_quat, current_quat))
        spin_pose = geometry_msg.Pose(current_pose.pose.position, spin_orientation)
        hdr = std_msg.Header(0, rospy.Time(0), '/map')
        spin_pose_st = geometry_msg.PoseStamped(hdr, spin_pose)
        rospy.loginfo('rotate %f', angle)
        rospy.loginfo('desired pose: %s', spin_pose_st)
        return spin_pose_st

    def turn_around(self, current_pose, start_pose):
        # spin in place to point at home
        around = tf.transformations.quaternion_from_euler(0,0,math.pi)
        q1 = (start_pose.pose.orientation.x, start_pose.pose.orientation.y,
                start_pose.pose.orientation.z, start_pose.pose.orientation.w)
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

    def see_beacon(self):
        self.beacon_pose = None
        yield teer_ros.WaitCondition(lambda: self.beacon_pose is not None)

    def twirl(self, other_task, do_star_twirl=False):
        self.move_base.cancel_goal()
        self.publish_zero_velocity()
        pose = self.get_current_robot_pose()
        q=(pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w)
        roll, pitch, yaw = tf_conversions.transformations.euler_from_quaternion(q)
        for i in range(9):
            # rotate a small amount
            q_goal = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw+2*math.pi*float(i)/8.0)
            pose.pose.orientation.x = q_goal[0]
            pose.pose.orientation.y = q_goal[1]
            pose.pose.orientation.z = q_goal[2]
            pose.pose.orientation.w = q_goal[3]
            yield teer_ros.WaitAnyTasks([
                self.new_task(self.drive_to_point(pose)),
                other_task ])

            # check to see which task finished
            move_state = self.move_base.get_state()
            if move_state in self.working_states:
                # in this case, other_task finished
                break

            if do_star_twirl:
                # move backwards a small amount
                yield teer_ros.WaitAnyTasks([
                    self.new_task(self.drive_to_point(self.forward(-1))),
                    other_task ])

                # check to see which task finished
                move_state = self.move_base.get_state()
                if move_state in self.working_states:
                    # in this case, other_task finished
                    break

                # move forward the same small amount
                yield teer_ros.WaitAnyTasks([
                    self.new_task(self.drive_to_point(self.forward(1))),
                    other_task ])

                # check to see which task finished
                move_state = self.move_base.get_state()
                if move_state in self.working_states:
                    # in this case, other_task finished
                    break

        self.move_base.cancel_goal()
        self.publish_zero_velocity()

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
        self.man_sample = None
        # wait for the manipulator camera to see the object
        yield teer_ros.WaitCondition(
                lambda: self.man_sample is not None)
        if self.man_sample is not None:
            self.announce("Sample in manipulator")
            # wait to stop
            self.move_base.cancel_goal()
            yield teer_ros.WaitDuration(0.5)

    def point_distance(self, pt1, pt2):
        return math.sqrt((pt1.x-pt2.x)**2 + (pt1.y-pt2.y)**2)

    def pursue(self, pose_st):
        goal = move_base_msg.MoveBaseGoal()
        goal.target_pose=pose_st
        self.move_base.send_goal(goal)

        self.search_sample = None
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
                rospy.loginfo("msg: %s", msg)
                newpt = self.listener.transformPoint(goal.target_pose.header.frame_id, msg)
                rospy.loginfo("newpt: %s", newpt)
                current_point = pose_st.pose.position
                if self.point_distance(current_point, newpt.point) > self.maximum_pursuit_error:
                    rospy.loginfo("new pursuit point %s", newpt)
                    pose_st.pose.position=newpt.point
                    goal.target_pose=pose_st
                    self.move_base.send_goal(goal)
                self.search_sample = None

    def drive_to_sample(self, start_pose):
        # switch control to the motion planner and drive to the expected
        # position
        self.drive_succeeded=False
        self.announce("Moving to expected sample location")
        self.platform_motion_input_select("Planner")

        # yield teer_ros.WaitTask(self.new_task(self.drive_to_point(self.forward(2))))
        # yield teer_ros.WaitTask(self.new_task(self.drive_to_point(self.rotate(20))))
        # yield teer_ros.WaitTask(self.new_task(self.drive_to_point(self.forward(5))))
        # yield teer_ros.WaitTask(self.new_task(self.drive_to_point(self.rotate(-40))))
        # yield teer_ros.WaitTask(self.new_task(self.drive_to_point(self.forward(1))))
        yield teer_ros.WaitTask(self.new_task(self.drive_to_point(self.forward(10))))
        self.pre_sample_search_pose = self.get_current_robot_pose()
        expected_position = self.forward(5)
        drive = self.new_task(self.drive_to_point(expected_position))
        find = self.new_task(self.wait_for_search_sample())
        yield teer_ros.WaitAnyTasks([drive, find])

        move_state = self.move_base.get_state()
        if move_state == action_msg.GoalStatus.SUCCEEDED and self.search_sample is None:
            # arrived at goal position without seeing sample, try to find it
            self.announce("Failed to see sample, twirling, twirling, twirling towards freedom")
            rospy.loginfo("Looking for sample")
            yield teer_ros.WaitTask(self.new_task(self.twirl(find)))
            self.kill_task(find)

        if self.search_sample is not None:
            # see sample in search, update goal and drive close
            self.kill_task(drive)
            old_search_sample = self.search_sample
            self.search_sample = None
            yield teer_ros.WaitCondition(lambda: self.search_sample is not None)
            if self.search_sample is None:
                self.search_sample = old_search_sample
            msg = self.search_sample
            self.listener.waitForTransform('/map', msg.header.frame_id,
                    msg.header.stamp, rospy.Duration(10.0))
            mappt = self.listener.transformPoint('/map', msg)
            expected_position.pose.position=mappt.point
            rospy.loginfo("pursue: %s", expected_position)
            pursue = self.new_task(self.pursue(expected_position))
            see = self.new_task(self.wait_for_manipulator_sample())
            yield teer_ros.WaitAnyTasks([pursue, see])
            if self.man_sample is None:
                self.announce("Failed to acquire sample in manipulator camera")
                yield teer_ros.WaitTask(self.new_task(self.twirl(see, True)))
                if self.man_sample is None:
                    self.kill_task(see)
                else:
                    self.drive_succeeded=True
            else:
                self.kill_task(pursue)
                self.drive_succeeded=True

    def acquire_sample(self, start_pose):
        # switch to visual servo control
        while True:
            self.announce("Sample detected, servoing")
            self.platform_motion_input_select("Servo")
            self.servo.send_goal(visual_servo_msg.VisualServoGoal(),
                                 feedback_cb=self.servo_feedback_cb
                                )
            lost_count = 0
            success = False
            while True:
                yield teer_ros.WaitDuration(0.1)
                state = self.servo.get_state()
                if state not in self.working_states:
                    success=True
                    break
                if self.lost_sample:
                    lost_count += 1
                else:
                    lost_count = 0
                if lost_count >= 200:
                    break
            if success:
                self.announce("Servo success, triggering manipulator")

                # grab the sample
                manip_goal = manipulator_msg.ManipulatorGoal()
                manip_goal.type=manip_goal.GRAB
                manip_goal.target_bin = 5
                manip_goal.grip_torque = 0.7
                manip_goal.wrist_angle = math.pi
                self.manipulator.send_goal(manip_goal)
                while True:
                    yield teer_ros.WaitDuration(0.1)
                    state = self.manipulator.get_state()
                    if state not in self.working_states:
                        break
                self.announce("Sample retreived")
            else:
                self.servo.cancel_goal()
                see = self.new_task(self.wait_for_manipulator_sample())
                yield self.new_task(self.twirl(see, True))
                if self.man_sample is None:
                    self.drive_home_task = self.new_task(self.drive_home(start_pose))
                    yield teer_ros.WaitTask(self.drive_home_task)
                    break
                else:
                    continue
            break

    def beacon_goal(self, current_beacon_point, new_beacon_pose, robot_pose):
        try:
            newpose = self.listener.transformPose('/map', new_beacon_pose)
        except Exception, e:
            rospy.logerr("failed to transform beacon pose: ", e)
            return
        rospy.loginfo("newpt: %s", newpose)
        dist = self.point_distance(
                current_beacon_point.pose.position,
                newpose.pose.position)
        if dist > self.maximum_pursuit_error:
            self.announce("aligning beacon")
            rospy.loginfo("dist: %f", dist)
            current_beacon_point.pose = newpose.pose
            current_beacon_point.header = newpose.header
            hdr = std_msg.Header(0, rospy.Time(0), '/map')
            b_pose = geometry_msg.PoseStamped()
            b_pose.header.stamp = rospy.Time(0)
            b_pose.header.frame_id = 'beacon'
            b_pose.pose.position.x = 0.0
            b_pose.pose.position.y = -1.3
            b_pose.pose.position.z = -1.0
            # robot upright in beacon frame
            q = tf.transformations.quaternion_from_euler(-math.pi/2, 0.0, -math.pi/2, 'rxyz')
            b_pose.pose.orientation.x = q[0]
            b_pose.pose.orientation.y = q[1]
            b_pose.pose.orientation.z = q[2]
            b_pose.pose.orientation.w = q[3]
            tfs = geometry_msg.TransformStamped()
            tfs.child_frame_id='beacon'
            tfs.header = new_beacon_pose.header
            tfs.transform.translation = new_beacon_pose.pose.position
            tfs.transform.rotation = new_beacon_pose.pose.orientation
            rospy.loginfo("inserting transform: %s", tfs)
            self.listener.setTransform(tfs, 'beacon_finder')
            g_pose = self.listener.transformPose('/map', b_pose)
            q=(g_pose.pose.orientation.x,g_pose.pose.orientation.y,g_pose.pose.orientation.z,g_pose.pose.orientation.w)
            roll, pitch, yaw = tf_conversions.transformations.euler_from_quaternion(q)
            q_upright = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw)
            g_pose.pose.orientation.x = q_upright[0]
            g_pose.pose.orientation.y = q_upright[1]
            g_pose.pose.orientation.z = q_upright[2]
            g_pose.pose.orientation.w = q_upright[3]
            rospy.loginfo("g: %s", g_pose)
            return g_pose
        return None


    def drive_home(self, start_pose):
        # spin in place to point at home
        self.platform_motion_input_select("Planner")
        capture_pose = self.get_current_robot_pose()
        spin_pose = self.turn_around(capture_pose.pose, self.pre_sample_search_pose)
        while True:
            yield teer_ros.WaitDuration(0.1)
            state = self.move_base.get_state()
            if state not in self.working_states:
                break
        rospy.loginfo("spin complete")

        # drive back the way we came from
        #yield teer_ros.WaitTask(self.new_task(self.drive_to_point(self.forward(3))))
        #yield teer_ros.WaitTask(self.new_task(self.drive_to_point(self.rotate(40))))

        # drive back to beacon
        home_goal = move_base_msg.MoveBaseGoal()
        home_goal.target_pose = self.home(start_pose.pose, spin_pose)
        self.move_base.send_goal(home_goal)
        current_beacon_point = home_goal.target_pose
        self.beacon_pose = None
        self.saw_beacon = False
        last_beacon_pose = self.get_current_robot_pose()
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
            beacon_dist = self.point_distance(last_beacon_pose.pose.position, robot_pose_st.pose.position)
            if not self.saw_beacon and beacon_dist>self.beacon_scan_distance:
                rospy.loginfo("looking for beacon")
                self.announce("Searching for beacon")
                see_beacon = self.new_task(self.see_beacon())
                yield teer_ros.WaitTask(self.new_task(self.twirl(see_beacon)))
                if self.beacon_pose is None:
                    self.move_base.send_goal(home_goal)
                self.saw_beacon = False
                last_beacon_pose = self.get_current_robot_pose()
            if self.beacon_pose is not None:
                last_beacon_pose = self.get_current_robot_pose()
                rospy.loginfo("new beacon")
                g_p_st = self.beacon_goal(current_beacon_point, self.beacon_pose, robot_pose_st)
                if g_p_st is not None:
                    home_goal.target_pose=g_p_st
                    self.move_base.send_goal(home_goal)
                else:
                    rospy.loginfo("current goal close enough")
                self.beacon_pose = None

        self.drive_home_task = None
        self.announce("Complete, waiting for mode change")
        mask = (self.GPIO_PIN_MODE_PRECACHED|self.GPIO_PIN_MODE_SEARCH)
        pin_states =\
          self.gpio.new_pin_states&mask
        yield teer_ros.WaitCondition(
                lambda: self.gpio.new_pin_states&mask != pin_states)
        rospy.loginfo("mode changed")

    def retrieve_precached_sample(self):
        # write down start pose
        start_pose = self.get_current_robot_pose()
        rospy.loginfo('start pose: %s', start_pose)

        self.drive_succeeded=False
        yield teer_ros.WaitTask(self.new_task(self.drive_to_sample(start_pose)))
        if self.drive_succeeded:
            yield teer_ros.WaitTask(self.new_task(self.acquire_sample(start_pose)))
        else:
            self.drive_home_task = self.new_task(self.drive_home(start_pose))
            yield teer_ros.WaitTask(self.drive_home_task)

def start_node():
    rospy.init_node('master_executive')
    sched = SampleReturnScheduler()
    # kick off startup task
    sched.new_task(sched.announcer())
    sched.new_task(sched.start_robot())
    sched.run()

if __name__=="__main__":
    start_node()
