import threading
import traceback
import collections
from copy import deepcopy

import rospy
import actionlib
import tf
import smach

import std_msgs.msg as std_msg
import geometry_msgs.msg as geometry_msg
import actionlib_msgs.msg as action_msg
import move_base_msgs.msg as move_base_msg
import samplereturn_msgs.msg as samplereturn_msg
import platform_motion_msgs.msg as platform_msg
import platform_motion_msgs.srv as platform_srv

import samplereturn.util as util

class MonitorTopicState(smach.State):
    """A state that checks a field in a given ROS topic, and compares against specified
    values.  Each specified value tiggers an outcome of the state.
    """
    def __init__(self, topic, field, msg_type, topic_value_outcomes, timeout=None):

        self._values = []
        #this is named outcome_list so as not to stomp
        #on the _outcomes variable in smach.State
        self._outcome_list = []
        self._thresholds = []
        self._value_counts = []

        for tvo in topic_value_outcomes:
            self._values.append(tvo[0])
            self._outcome_list.append(tvo[1])
            self._thresholds.append(tvo[2])
            self._value_counts.append(0)

        self._outcome_list.append('timeout')
        self._outcome_list.append('preempted')

        smach.State.__init__(self, outcomes=self._outcome_list)

        self._topic = topic
        self._field = field
        self._msg_type = msg_type
        self._timeout = timeout
        self._triggered_outcome = None

        self._trigger_cond = threading.Condition()

    def execute(self, ud):

        self._sub = rospy.Subscriber(self._topic, self._msg_type, self._sub_cb)

        self._trigger_cond.acquire()
        self._trigger_cond.wait(timeout=self._timeout)
        self._trigger_cond.release()

        self._sub.unregister()

        if self._triggered_outcome is not None:
            return self._triggered_outcome

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        return 'timeout'

    #the topic subscriber callback
    def _sub_cb(self, msg):
        if self._value_check(getattr(msg, self._field)):
            self._trigger_cond.acquire()
            self._trigger_cond.notify()
            self._trigger_cond.release()

    #returns true if one of the values received meets an outcome threshold, else, false
    def _value_check(self, check_value):
        if check_value in self._values:
            i = self._values.index(check_value)
            self._value_counts[i] += 1
            if (self._value_counts[i] >= self._thresholds[i]):
                self._triggered_outcome = self._outcome_list[i]
                return True

        return False

    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_cond.acquire()
        self._trigger_cond.notify()
        self._trigger_cond.release()

#state that waits for a userdata flag to change, this must be
#done outside the state machine... which is maybe cool
class WaitForFlagState(smach.State):

    def __init__(self,
                 flag_name,
                 flag_trigger_value=True,
                 timeout=0,
                 announcer=None,
                 start_message=None):

        smach.State.__init__(self,
                             outcomes=['next', 'timeout', 'aborted'],
                             input_keys=[flag_name])

        self.flag_name = flag_name
        self.flag_trigger_value = flag_trigger_value
        self.timeout = timeout
        self.announcer = announcer
        self.start_message = start_message

    def execute(self, userdata):

        if getattr(userdata, self.flag_name) != self.flag_trigger_value and \
           (self.announcer is not None) and \
           (self.start_message is not None):
            self.announcer.say(self.start_message)

        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            if getattr(userdata, self.flag_name) == self.flag_trigger_value:
                return 'next'
            rospy.sleep(0.1)
            if (self.timeout > 0) and ((rospy.get_time() - start_time) > self.timeout):
                return 'timeout'

        return 'paused'

#general class for using the robot's planner to drive to a point
class DriveToPoseState(smach.State):

    def __init__(self, move_client, listener):

        smach.State.__init__(self,
                             outcomes=['complete',
                                       'timeout',
                                       'sample_detected',
                                       'preempted','aborted'],
                             input_keys=['target_pose',
                                         'velocity',
                                         'pursue_samples',
                                         'stop_on_sample',
                                         'detected_sample',
                                         'motion_check_interval',
                                         'min_motion',
                                         'paused'],
                             output_keys=['last_pose',
                                          'detected_sample'])

        self.move_client = move_client

        self.listener = listener
        self.sample_detected = False

    def execute(self, ud):
        #on entry to Drive To Pose clear old sample_detections!
        ud.detected_sample = None
        last_pose = util.get_current_robot_pose(self.listener)
        velocity = ud.velocity
        goal = move_base_msg.MoveBaseGoal()
        goal.target_pose=ud.target_pose
        self.move_client.send_goal(goal)
        rospy.loginfo("DriveToPose initial goal: %s" % (goal))
        move_state = self.move_client.get_state()
        last_motion_check_time = rospy.get_time()
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            #log current pose for use in other states
            current_pose = util.get_current_robot_pose(self.listener)
            ud.last_pose = current_pose
            #is action server still working?
            move_state = self.move_client.get_state()
            if move_state not in util.actionlib_working_states:
                break
            #handle preempts
            if self.preempt_requested():
                rospy.loginfo("PREEMPT REQUESTED IN DriveToPoseState")
                self.move_client.cancel_all_goals()
                self.service_preempt()
                return 'preempted'
            #handle sample detection
            if (ud.detected_sample is not None) and ud.pursue_samples:
                if ud.stop_on_sample:
                    self.move_client.cancel_all_goals()
                return 'sample_detected'
            #handle target_pose changes
            if ud.target_pose != goal.target_pose:
                rospy.loginfo("DriveToPose replanning")
                goal.target_pose = ud.target_pose
                self.move_client.send_goal(goal)
            #check if we are stuck and move_base isn't handling it
            now = rospy.get_time()
            if (now - last_motion_check_time) > ud.motion_check_interval:
                distance = util.pose_distance_2d(current_pose, last_pose)
                last_pose = current_pose
                last_motion_check_time = now
                if (distance < ud.min_motion):
                    self.move_client.cancel_all_goals()
                    return 'timeout'
            #if we are paused, cancel goal and wait, then resend
            #last goal and reset timeout timer
            if ud.paused:
                self.move_client.cancel_all_goals()
                while not rospy.is_shutdown():
                    rospy.sleep(0.2)
                    if not ud.paused:
                        break
                self.move_client.send_goal(goal)
                self.last_motion_check_time = rospy.get_time()
                    
        if move_state == action_msg.GoalStatus.SUCCEEDED:
            return 'complete'

        return 'aborted'

#pursues a detected point, changing the move_base goal if the detection point
#changes too much.  Detection and movement all specified in executive_frame.
#exits if pose is within min_pursuit_distance, and sets target_pose to the current
#detection point with current robot orientation
class PursuePointManager(smach.State):

    def __init__(self, listener):

        smach.State.__init__(self,
                             outcomes=['min_distance',
                                       'point_lost',
                                       'preempted', 'aborted'],
                             input_keys=['target_pose',
                                         'pursuit_point',
                                         'last_pose',
                                         'min_pursuit_distance',
                                         'max_pursuit_error',
                                         'max_point_lost_time',
                                         'executive_frame'],
                             output_keys=['target_pose'])

        self.listener = listener

    def execute(self, userdata):

        last_point_detection = userdata.pursuit_point.header.stamp

        while not rospy.is_shutdown():
            #last_pose is updated by DriveToPose
            current_pose = userdata.last_pose
            point_distance = util.pose_distance_2d(current_pose, userdata.target_pose)
            rospy.logdebug("PURSUIT distance: " + str(point_distance))
            if point_distance < userdata.min_pursuit_distance:
                return 'min_distance'

            #has pursuit point been changed outside the state?
            #check with the time stamp.
            pursuit_point = userdata.pursuit_point
            if pursuit_point is not None and \
             (pursuit_point.header.stamp != last_point_detection):
                last_point_detection = pursuit_point.header.stamp
                point_stamped = deepcopy(userdata.pursuit_point)

                #if so, has it moved more the max pursuit error?
                #compare the transformed input point to the point in target_pose
                point_delta = util.point_distance_2d(userdata.target_pose.pose.position,
                                                     point_stamped.point)
                if point_delta > userdata.max_pursuit_error:
                    rospy.loginfo("POINT_DELTA: " + str(point_delta))
                    rospy.logdebug("PURSUE point updated: %s", point_stamped)
                    new_pose = deepcopy(userdata.target_pose)
                    new_pose.pose.position = point_stamped.point
                    userdata.target_pose = new_pose

            if (rospy.Time.now() - last_point_detection) > \
             rospy.Duration(userdata.max_point_lost_time):
                return 'point_lost'

            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'

            rospy.sleep(0.2)

        return 'aborted'

#the class that contains both the pursuit manager and the move_base handling state
class PursueDetectedPoint(smach.Concurrence):

    def __init__(self, listener):

        smach.Concurrence.__init__(self,
            outcomes=['min_distance',
                      'complete',
                      'timeout',
                      'point_lost',
                      'preempted', 'aborted'],
            default_outcome = 'aborted',
            input_keys=['target_pose',
                        'last_pose',
                        'pursuit_point',
                        'pursue_samples',
                        'stop_on_sample',
                        'detected_sample',
                        'velocity',
                        'min_pursuit_distance',
                        'max_pursuit_error',
                        'max_point_lost_time',
                        'motion_check_interval',
                        'min_motion',
                        'paused',
                        'executive_frame'],
            output_keys=['target_pose',
                         'last_pose'],
            child_termination_cb = lambda preempt: True,
            outcome_map = {'min_distance':{'PURSUIT_MANAGER':'min_distance'},
                           'point_lost':{'PURSUIT_MANAGER':'point_lost'},
                           'complete':{'DRIVE_TO_POSE':'complete'},
                           'timeout':{'DRIVE_TO_POSE':'timeout'}})

        self.listener = listener

    def execute(self, userdata):

        #Get start pose and pursuit point, calculate the first target_pose
        #and put it in the userdata.  Put start_pose into last_pose as well
        #Then, execute concurrence as normal.
        goal_header = std_msg.Header(0, rospy.Time(0), userdata.executive_frame)
        start_pose = util.get_current_robot_pose(self.listener)
        rospy.logdebug("PURSUIT start pose: " + str(start_pose))
        point_stamped = userdata.pursuit_point
        goal_pose = geometry_msg.Pose(point_stamped.point,
                    util.pointing_quaternion_2d(start_pose.pose.position,
                                                point_stamped.point))
        userdata.target_pose = geometry_msg.PoseStamped(goal_header, goal_pose)
        userdata.last_pose = start_pose
        rospy.logdebug("PURSUIT initial target point: %s", point_stamped)

        return smach.Concurrence.execute(self, userdata)

def GetPursueDetectedPointState(move_client, listener):

    pursue = PursueDetectedPoint(listener)
    with pursue:
        smach.Concurrence.add('PURSUIT_MANAGER',
                              PursuePointManager(listener))
        smach.Concurrence.add('DRIVE_TO_POSE',
                              DriveToPoseState(move_client, listener))

    return pursue

class SelectMotionMode(smach.State):
    def __init__(self, CAN_interface, announcer, motion_mode, failannounce=None):
        smach.State.__init__(self, outcomes = ['next', 'failed'])
        self.CAN_interface = CAN_interface
        self.announcer = announcer
        self.motion_mode = motion_mode
        self.failannounce = failannounce

    def execute(self, userdata):
        try:
            mode = self.CAN_interface.select_mode(platform_srv.SelectMotionModeRequest.MODE_QUERY)
            if mode.mode == self.motion_mode:
                return 'next'
        except rospy.ServiceException:
            rospy.logerr( "Unable to query present motion mode")
            self.announcer.say( "Unable to query present motion mode." )
        try:
            self.CAN_interface.select_mode(self.motion_mode)
        except rospy.ServiceException:
            rospy.logerr( "Unable to select mode %d", self.motion_mode )
            if self.failannounce is not None:
                rospy.logerr( self.failannounce )
                self.announcer.say(self.failannounce)
            return 'failed'
        self.CAN_interface.publish_zero()
        return 'next'

class AnnounceState(smach.State):
    def __init__(self, announcer, announcement):
        smach.State.__init__(self, outcomes = ['next'])
        self.announcer = announcer
        self.announcement = announcement
    def execute(self, userdata):
        self.announcer.say(self.announcement)
        return 'next'


