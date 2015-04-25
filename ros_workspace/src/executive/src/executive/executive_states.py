import threading
import traceback
import collections
from copy import deepcopy
import numpy as np

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
from motion_planning.simple_motion import TimeoutException
from samplereturn_msgs.msg import SimpleMoveGoal
from samplereturn_msgs.msg import (VFHMoveAction,
                                   VFHMoveResult,
                                   VFHMoveFeedback)

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

    def execute(self, userdata):

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
                             outcomes=['next', 'timeout', 'preempted', 'aborted'],
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
        while True:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'            
            if getattr(userdata, self.flag_name) == self.flag_trigger_value:
                return 'next'
            rospy.sleep(0.1)
            if (self.timeout > 0) and ((rospy.get_time() - start_time) > self.timeout):
                return 'timeout'

        return 'paused'

#general action server
class ExecuteMoveState(smach.State):
    
    def __init__(self, move_client, **kwargs):
        
        super(ExecuteMoveState, self).__init__(**kwargs)    
    
        self._move_client = move_client
        
    #handles pause
    def check_pause(self, userdata, goal):
        #Check to see if we are paused.  If so, cancel active goals.
        #Resend last goal on unpause
        if userdata.paused:
            self.cancel_move()
            while True:
                rospy.sleep(0.1)
                #Must check preempt in pause loop too, no matter how lame it is
                #If preempted here, exit, and let main loop handle preempt
                #Do not resend goal! Only resend goal on unpause
                if self.preempt_requested(): return 
                if not userdata.paused: break
            self._move_client.send_goal(goal)      
    
    #handles preempts
    def handle_preempt(self):
        rospy.loginfo("{!s}: preempt requested".format(self.__class__.__name__))
        self.cancel_move()
        self.service_preempt()
        return 'preempted'
    
    #this method cancels the action server move, and waits for motion to 
    def cancel_move(self):
        name = self.__class__.__name__
        move_state = self._move_client.get_state()
        self._move_client.cancel_all_goals()
        start_time = rospy.get_time()
        while True:
            rospy.sleep(0.1) 
            #if we are in shutdown, stop polling the action server
            if rospy.core.is_shutdown_requested():
                return
            elapsed = rospy.get_time() - start_time
            move_state = self._move_client.get_state()
            if move_state not in util.actionlib_working_states:
                rospy.loginfo("{!s}: action server reports stopped (no longer working state) in: {:f} seconds".format(name, elapsed))
                return
            if  elapsed > 5.0:
                rospy.logwarn("{!s}: action server failed to report stop with 5 seconds after cancel request".format(name))
                return
        return    

#general class for using the robot's planner to drive to a point
class ExecuteVFHMove(ExecuteMoveState):

    def __init__(self, move_client):

        super(ExecuteVFHMove, self).__init__(move_client,
                                            outcomes=['complete',
                                                      'blocked',
                                                      'started_blocked',
                                                      'off_course',
                                                      'missed_target',
                                                      'sample_detected',
                                                      'preempted','aborted'],
                                            input_keys=['move_goal',
                                                        'stop_on_sample',
                                                        'detected_sample',
                                                        'paused',
                                                        'odometry_frame'],
                                            output_keys=['detected_sample',
                                                         'vfh_result'])
      
    def execute(self, userdata):
        #on entry clear old sample_detections!
        userdata.detected_sample = None
        userdata.vfh_result = None
        goal = deepcopy(userdata.move_goal)
        rospy.loginfo("ExecuteVFHMove initial goal: %s" % (goal))
        self._move_client.send_goal(goal)
        while True:
            #Wait, then get action server state.
            rospy.sleep(0.1)
            #First, handle preempts.
            if self.preempt_requested():
                return self.handle_preempt()
            #Is action server still working?
            move_state = self._move_client.get_state()
            if move_state not in util.actionlib_working_states:
                break
            #Handle sample detection
            if (userdata.detected_sample is not None) and userdata.stop_on_sample:
                rospy.loginfo("ExecuteVFHMove detected sample: " + str(userdata.detected_sample))
                self.cancel_move()
                return 'sample_detected'
            #handle goal changes
            if not (goal.target_pose == userdata.move_goal.target_pose):
                rospy.loginfo("ExecuteVFHMove goal changed: {!s} to {!s}".format(goal, userdata.move_goal))
                goal = deepcopy(userdata.move_goal)
                self._move_client.send_goal(goal)
            #checks to see if pause requested, and handles this
            self.check_pause(userdata, goal)
            
        if (move_state == action_msg.GoalStatus.SUCCEEDED):
            if (self._move_client.wait_for_result(timeout = rospy.Duration(2.0))):
                result = self._move_client.get_result().outcome
                userdata.vfh_result = result
                if result == VFHMoveResult.COMPLETE:
                    return 'complete'
                elif result == VFHMoveResult.BLOCKED:
                    return 'blocked'
                elif result == VFHMoveResult.STARTED_BLOCKED:
                    return 'started_blocked'
                elif result == VFHMoveResult.OFF_COURSE:
                    return 'off_course'
                elif result == VFHMoveResult.MISSED_TARGET:
                    return 'missed_target'
                else:
                    rospy.logwarn("ExecuteVFHMove received unexpected outcome from action server, aborting")
                    return 'aborted'
            else:
                rospy.logwarn("ExecuteVFHMove received SUCCEEDED state from action server, but no action result, aborting")

        self.cancel_move()
        return 'aborted'
       

#simple move executor
class ExecuteSimpleMove(ExecuteMoveState):
    def __init__(self, move_client):
        
        super(ExecuteSimpleMove, self).__init__(move_client,
                                                outcomes=['complete',
                                                          'sample_detected',
                                                          'preempted',
                                                          'aborted'],
                                                input_keys=['simple_move',
                                                            'stop_on_sample',
                                                            'detected_sample',
                                                            'paused'],
                                                output_keys=['detected_sample'])
        
    def execute(self, userdata):

        userdata.detected_sample = None
        goal = userdata.simple_move
        self._move_client.send_goal(goal)

        #watch the action server
        while True:
            rospy.sleep(0.1)
            move_state = self._move_client.get_state()
            if move_state not in util.actionlib_working_states:
                break            
            if self.preempt_requested():
                return self.handle_preempt()
            #Handle sample detection
            if userdata.stop_on_sample and (userdata.detected_sample is not None):
                rospy.loginfo("ExecuteSimpleMove detected sample: " + str(userdata.detected_sample))
                self.cancel_move()
                return 'sample_detected'
            #if we are paused, cancel goal and wait, then resend
            #last goal and reset timeout timer
            self.check_pause(userdata, goal)
               
        if move_state == action_msg.GoalStatus.SUCCEEDED:
            return 'complete'                

        return 'aborted'

class SelectMotionMode(smach.State):
    def __init__(self, CAN_interface, motion_mode):
        smach.State.__init__(self, outcomes = ['next', 'paused', 'failed'])
        self.CAN_interface = CAN_interface
        self.motion_mode = motion_mode

    def execute(self, userdata):
        valid_pause_transitions = [platform_srv.SelectMotionModeRequest.MODE_RESUME,
                                   platform_srv.SelectMotionModeRequest.MODE_ENABLE,
                                   platform_srv.SelectMotionModeRequest.MODE_LOCK,
                                   platform_srv.SelectMotionModeRequest.MODE_PAUSE]
        try:
            current_mode = self.CAN_interface.select_mode(platform_srv.SelectMotionModeRequest.MODE_QUERY)
            if current_mode.mode == self.motion_mode:
                return 'next'
            elif current_mode.mode == platform_srv.SelectMotionModeRequest.MODE_PAUSE \
                    and self.motion_mode not in valid_pause_transitions:
                return 'paused'
        except rospy.ServiceException:
            rospy.logerr( "Unable to query present motion mode")
            return 'failed'
        
        try:
            self.CAN_interface.select_mode(self.motion_mode)
        except rospy.ServiceException:
            rospy.logerr( "Unable to select mode %d", self.motion_mode )
            return 'failed'

        return 'next'

#class for saying something on the speakers
class AnnounceState(smach.State):
    def __init__(self, announcer, announcement):
        smach.State.__init__(self,
                             outcomes = ['next'],
                             input_keys = ['announcement'])
        self.announcer = announcer
        self.announcement = announcement
    def execute(self, userdata):
        if self.announcement is None:
            self.announcement = userdata.announcement
        self.announcer.say(self.announcement)
        return 'next'

#class for servoing robot onto a sample using manipulator detections
class ServoController(smach.State):
    def __init__(self, tf_listener, announcer):
        smach.State.__init__(self,
                             outcomes=['move', 'complete', 'point_lost', 'aborted'],
                             input_keys=['detected_sample',
                                         'manipulator_correction',
                                         'servo_params',
                                         'simple_move'],
                             output_keys=['simple_move',
                                          'detected_sample',
                                          'latched_sample',
                                          'stop_on_sample'])
        
        self.tf_listener = tf_listener
        self.announcer = announcer
        self.try_count = 0
        
    def execute(self, userdata):
        
        userdata.stop_on_sample = False
        userdata.detected_sample = None
        rospy.sleep(userdata.servo_params['settle_time'])
        if self.try_count > userdata.servo_params['try_limit']:
            self.announcer.say("Servo exceeded try limit")
            rospy.loginfo("SERVO STRAFE failed to hit tolerance before try_limit: %s" % (userdata.servo_params['try_limit']))
            self.try_count = 0
            return 'complete'
        
        if userdata.detected_sample is None:
            self.announcer.say("Sample lost")
            self.try_count = 0
            return 'point_lost'
        else:
            sample_time = userdata.detected_sample.header.stamp
            sample_frame = userdata.detected_sample.header.frame_id
            while not rospy.core.is_shutdown_requested():
                try:
                    self.tf_listener.waitForTransform('manipulator_arm',
                            sample_frame, sample_time, rospy.Duration(1.0))
                    point_in_manipulator = self.tf_listener.transformPoint('manipulator_arm',
                                                                 userdata.detected_sample).point
                    break
                except tf.Exception, exc:
                    rospy.logwarn("SERVO CONTROLLER failed to get manipulator_arm -> {!s} transform.  Exception: {!s}".format(sample_frame, exc))
                    self.try_count = 0
                    return 'aborted'
            #rospy.loginfo("SERVO correction %s: " % (userdata.manipulator_correction))
            rospy.loginfo("SERVO point in manipulator before correction %s: " %(point_in_manipulator))
            point_in_manipulator.x += userdata.manipulator_correction['x']
            point_in_manipulator.y += userdata.manipulator_correction['y']
            #rospy.loginfo("SERVO point in manipulator after correction %s: " %(point_in_manipulator))
            origin = geometry_msg.Point(0,0,0)
            distance = util.point_distance_2d(origin, point_in_manipulator)
            if distance < userdata.servo_params['final_tolerance']:
                self.try_count = 0
                userdata.latched_sample = userdata.detected_sample
                self.announcer.say("Servo complete at %.1f millimeters." % (distance*1000))    
                return 'complete'
            elif distance < userdata.servo_params['initial_tolerance']:
                velocity = userdata.servo_params['final_velocity']
                accel = userdata.servo_params['final_accel']
            else:
                velocity = userdata.servo_params['initial_velocity']
                accel = userdata.servo_params['initial_accel']
            yaw = util.pointing_yaw(origin, point_in_manipulator)
            userdata.simple_move =  SimpleMoveGoal(type=SimpleMoveGoal.STRAFE,
                                                   angle = yaw,
                                                   distance = distance,
                                                   velocity = velocity,
                                                   acceleration = accel)
            #rospy.loginfo("SERVO simple_move: %s" % (userdata.simple_move))
            self.announcer.say("Servo ing, distance, %.1f centimeters" % (distance*100))
            rospy.loginfo("DETECTED SAMPLE IN manipulator_arm frame (corrected): %s" % (point_in_manipulator))
            self.try_count += 1
            return 'move'
        
        self.try_count = 0
        return 'aborted'


#scary class for going to list of points with no obstacle checking,
#good for searching area with strafing, or complex mounting maneuvers
class MoveToPoints(smach.State):
    def __init__(self, tf_listener):
        smach.State.__init__(self,
                             input_keys=['point_list',
                                         'detected_sample',
                                         'odometry_frame',
                                         'stop_on_sample',
                                         'velocity'],
                             output_keys=['simple_move',
                                          'point_list'],
                             outcomes=['next_point',
                                       'complete',
                                       'aborted'])
        
        self.tf_listener = tf_listener
            
    def execute(self, userdata):    
               
        if (len(userdata.point_list) > 0):
            rospy.loginfo("MOVE TO POINTS list: %s" %(userdata.point_list))
            target_point = userdata.point_list[0]
            header = std_msg.Header(0, rospy.Time(0), userdata.odometry_frame)
            target_stamped = geometry_msg.PointStamped(header, target_point)
            try:
                yaw, distance = util.get_robot_strafe(self.tf_listener, target_stamped)
            except(tf.Exception):
                rospy.logwarn("MOVE_TO_POINTS failed to transform search point (%s) to base_link in 1.0 seconds", search_point.header.frame_id)
                return 'aborted'

            userdata.point_list.popleft()
            userdata.simple_move = SimpleMoveGoal(type=SimpleMoveGoal.STRAFE,
                                                  angle = yaw,
                                                  distance = distance,
                                                  velocity = userdata.velocity)
            return 'next_point'                

        else:
            return 'complete'
        
        return 'aborted'

