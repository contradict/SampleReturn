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

#general class for using the robot's planner to drive to a point
class ExecuteVFHMove(smach.State):

    def __init__(self, move_client):

        smach.State.__init__(self,
                             outcomes=['complete',
                                       'sample_detected',
                                       'preempted','aborted'],
                             input_keys=['move_goal',
                                         'pursue_samples',
                                         'stop_on_sample',
                                         'detected_sample',
                                         'paused',
                                         'odometry_frame',
                                         'shutdown_requested'],
                             output_keys=['detected_sample'])

        self.move_client = move_client

    def execute(self, userdata):
        #on entry clear old sample_detections!
        userdata.detected_sample = None
        goal = userdata.move_goal
        rospy.loginfo("ExecuteVFHMove initial goal: %s" % (goal))
        self.move_client.send_goal(goal)
        while True:
            #Wait, then get action server state.
            rospy.sleep(0.1)
            #First, handle preempts.
            if self.preempt_requested():
                return self.handle_preempt()
            #Is action server still working?
            move_state = self.move_client.get_state()
            if move_state not in util.actionlib_working_states:
                break
            #Handle sample detection
            if (userdata.detected_sample is not None) and userdata.pursue_samples:
                rospy.loginfo("ExecuteVFHMove detected sample: " + str(userdata.detected_sample))
                if userdata.stop_on_sample:
                    self.cancel_move()
                return 'sample_detected'
            #Check to see if we are paused.  If so, cancel active goals.
            #Resend last goal on unpause
            if userdata.paused:
                self.cancel_move()
                while True:
                    rospy.sleep(0.1)
                    #Must check preempt in pause loop too, no matter how lame it is
                    if self.preempt_requested():
                        return self.handle_preempt()
                    if not userdata.paused: break
                self.move_client.send_goal(goal)      
                                    
        if move_state == action_msg.GoalStatus.SUCCEEDED:
            return 'complete'

        self.cancel_move()
        return 'aborted'

    def handle_preempt(self):
        rospy.loginfo("ExecuteVFHMove: preempt requested")
        self.cancel_move()
        self.service_preempt()
        return 'preempted'
    
    #this method cancels the action server move, and waits for motion to 
    def cancel_move(self):
        move_state = self.move_client.get_state()
        self.move_client.cancel_all_goals()
        start_time = rospy.get_time()
        while True:
            rospy.sleep(0.1) 
            #if we are in shutdown, stop polling the action server
            if rospy.core.is_shutdown_requested():
                return
            elapsed = rospy.get_time() - start_time
            move_state = self.move_client.get_state()
            if move_state not in util.actionlib_working_states:
                rospy.loginfo("ExecuteVFHMove: action server reports stopped (no longer working state) in: %f seconds" % elapsed)
                return
            if  elapsed > 5.0:
                rospy.logwarn("ExecuteVFHMove: action server failed to report stop with 5 seconds after cancel request")
                return
        return
        

#simple move executor, set up interrupts outside this.  If you want obstacle detection
#to work, and this is strafing in a checked direction, set the active_strafe_key
class ExecuteSimpleMove(smach.State):
    def __init__(self, move_client):
        
        smach.State.__init__(self,
                             outcomes=['complete',
                                       'sample_detected',
                                       'timeout',
                                       'preempted',
                                       'aborted'],
                             input_keys=['simple_move',
                                         'stop_on_sample',
                                         'detected_sample',
                                         'paused'],
                             output_keys=['simple_move'])
        
        self.move_client = move_client
        
    def execute(self, userdata):

        goal = userdata.simple_move
        self.move_client.send_goal(goal)

        #watch the action server
        while True:
            rospy.sleep(0.1)
            move_state = self.move_client.get_state()
            if move_state not in util.actionlib_working_states:
                break            
            if self.preempt_requested():
                self.move_client.cancel_all_goals()
                self.service_preempt()
                return 'preempted'
            #Handle sample detection
            if userdata.stop_on_sample and (userdata.detected_sample is not None):
                rospy.loginfo("ExecuteSimpleMove detected sample: " + str(userdata.detected_sample))
                self.move_client.cancel_all_goals()
                return 'sample_detected'
            #if we are paused, cancel goal and wait, then resend
            #last goal and reset timeout timer
            if userdata.paused:
                self.move_client.cancel_all_goals()
                while True():
                    rospy.sleep(0.1)
                    if self.preempt_requested():
                        self.service_preempt()
                        return 'preempted'
                    if not userdata.paused: break
                self.move_client.send_goal(goal)
                
        if move_state == action_msg.GoalStatus.SUCCEEDED:
            return 'complete'                

        return 'aborted'


#pursues a detected point, changing the move_base goal if the detection point
#changes too much.  Detection and movement all specified in odometry_frame.
#exits if pose is within min_pursuit_distance, and sets target_pose to the current
#detection point with current robot orientation
class PursuePointManager(smach.State):

    def __init__(self, listener):

        smach.State.__init__(self,
                             outcomes=['min_distance',
                                       'point_lost',
                                       'preempted', 'aborted'],
                             input_keys=['move_goal',
                                         'pursuit_point',
                                         'min_pursuit_distance',
                                         'max_pursuit_error',
                                         'max_sample_lost_time',
                                         'odometry_frame'],
                             output_keys=['move_goal'])

        self.listener = listener

    def execute(self, userdata):

        last_point_detection = userdata.pursuit_point.header.stamp

        while True:
            try:
                current_pose = util.get_current_robot_pose(self.listener,
                        userdata.odometry_frame)
            except(tf.Exception):
                rospy.logwarn("PursuePointManager failed to get current pose")
                return 'aborted'
            point_distance = util.pose_distance_2d(current_pose,
                                                   userdata.move_goal.target_pose)
            rospy.logdebug("PURSUIT distance: " + str(point_distance))
            if point_distance < userdata.min_pursuit_distance:
                rospy.loginfo("PURSUE_POINT achieved min_distance: %f" % point_distance)
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
             rospy.Duration(userdata.max_sample_lost_time):
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
                      'point_lost',
                      'preempted', 'aborted'],
            default_outcome = 'aborted',
            input_keys=['move_goal',
                        'pursuit_point',
                        'pursue_samples',
                        'stop_on_sample',
                        'detected_sample',
                        'min_pursuit_distance',
                        'max_pursuit_error',
                        'max_sample_lost_time',
                        'paused',
                        'odometry_frame'],
            output_keys=['move_goal'],
            child_termination_cb = lambda preempt: True,
            outcome_map = {'min_distance':{'PURSUIT_MANAGER':'min_distance'},
                           'point_lost':{'PURSUIT_MANAGER':'point_lost'},
                           'complete':{'EXECUTE_MOVE':'complete'},
                           'aborted':{'PURSUIT_MANAGER':'preempted',
                                        'EXECUTE_MOVE':'preempted'}})

        self.listener = listener

    def execute(self, userdata):

        #Get start pose and pursuit point, calculate the first target_pose
        #and put it in the userdata.  Put start_pose into last_pose as well
        #Then, execute concurrence as normal.
        goal_header = std_msg.Header(0, rospy.Time(0), userdata.odometry_frame)
        try:
            start_pose = util.get_current_robot_pose(self.listener,
                    userdata.odometry_frame)
        except (tf.Exception):
            rospy.logwarn("PursueDetectedPoint failed to get current pose")
            return 'aborted'
        rospy.logdebug("PURSUIT start pose: " + str(start_pose))
        point_stamped = userdata.pursuit_point
        goal_pose = geometry_msg.Pose(point_stamped.point,
                    util.pointing_quaternion_2d(start_pose.pose.position,
                                                point_stamped.point))
        target_pose = geometry_msg.PoseStamped(goal_header, goal_pose)
        goal = samplereturn_msg.VFHMoveGoal(target_pose = target_pose)
        userdata.move_goal = goal
        rospy.logdebug("PURSUIT initial target point: %s", point_stamped)

        return smach.Concurrence.execute(self, userdata)

def GetPursueDetectedPointState(move_client, listener):

    pursue = PursueDetectedPoint(listener)
    with pursue:
        smach.Concurrence.add('PURSUIT_MANAGER',
                              PursuePointManager(listener))
        smach.Concurrence.add('EXECUTE_MOVE',
                              ExecuteVFHMove(move_client))

    return pursue

class SelectMotionMode(smach.State):
    def __init__(self, CAN_interface, motion_mode):
        smach.State.__init__(self, outcomes = ['next', 'paused', 'failed'])
        self.CAN_interface = CAN_interface
        self.motion_mode = motion_mode

    def execute(self, userdata):
        try:
            current_mode = self.CAN_interface.select_mode(platform_srv.SelectMotionModeRequest.MODE_QUERY)
            if current_mode.mode == self.motion_mode:
                return 'next'
            elif current_mode.mode == platform_srv.SelectMotionModeRequest.MODE_PAUSE \
                    and self.motion_mode != platform_srv.SelectMotionModeRequest.MODE_RESUME:
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

class AnnounceState(smach.State):
    def __init__(self, announcer, announcement):
        smach.State.__init__(self, outcomes = ['next'])
        self.announcer = announcer
        self.announcement = announcement
    def execute(self, userdata):
        self.announcer.say(self.announcement)
        return 'next'

class ServoController(smach.State):
    def __init__(self, tf_listener, announcer):
        smach.State.__init__(self,
                             outcomes=['move', 'complete', 'point_lost', 'aborted'],
                             input_keys=['detected_sample',
                                         'settle_time',
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
            try:
                self.tf_listener.waitForTransform('manipulator_arm',
                        sample_frame, sample_time, rospy.Duration(1.0))
                point_in_manipulator = self.tf_listener.transformPoint('manipulator_arm',
                                                             userdata.detected_sample).point
            except tf.Exception:
                rospy.logwarn("SERVO CONTROLLER failed to get manipulator_arm -> odom transform in 1.0 seconds")
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
    
class RotateToClear(smach.State):
    def __init__(self, simple_mover, tf_listener):
        smach.State.__init__(self,
                             input_keys=['clear_spin',
                                         'clear_move',
                                         'strafes',
                                         'paused'],
                             output_keys=['active_strafe_key',
                                          'point_list'],
                             outcomes=['complete',
                                       'blocked',
                                       'aborted'])
        
        self.tf_listener = tf_listener
        self.simple_mover = simple_mover
        self.clear = False
        self.strafes = None #this may be useful later
 
        rospy.Subscriber('costmap_check',
                          samplereturn_msg.CostmapCheck,
                          self.handle_costmap_check)
 
    def execute(self, userdata):
        
        move = deepcopy(userdata.clear_spin)
        self.clear = False
        
        #load values from dict, absent values become None
        if move['type'] != 'spin':
            rospy.logwarn('ROTATE TO CLEAR received non-spin simple move')
            return 'aborted'
        
        angle = move.get('angle')
        velocity = move.get('velocity')
        accel = move.get('acceleration')
 
        while not rospy.is_shutdown():
            try:
                error = self.simple_mover.execute_spin(angle,
                                                       max_velocity = velocity,
                                                       acceleration = accel,
                                                       stop_function = self.stop_if_clear)
                rospy.loginfo("EXECUTED ROTATE TO CLEAR: %.1f, error %.3f" %( np.degrees(angle),
                                                                              np.degrees(error)))
                #did we exit the move execute because of a pause?
                if userdata.paused:
                    #wait here for unpause, as long as it takes
                    rospy.loginfo("ROTATE TO CLEAR stopped by pause")
                    while not rospy.is_shutdown() and userdata.paused:
                        rospy.sleep(0.2)
                    #unpaused, try again, changing both goal values to returned error
                    angle = error
                else:
                    #made it through move without being paused, break out
                    break
                
            except(TimeoutException):
                    rospy.logwarn("TIMEOUT during simple_motion.")
                    return 'timeout'

        if self.clear:
            
            rospy.sleep(3.0) #costmap wait

            move = deepcopy(userdata.clear_move)
            self.clear = False
 
            angle = move.get('angle')
            distance = move.get('distance')
            velocity = move.get('velocity')
            accel = move.get('acceleration')

            while not rospy.is_shutdown():
                try:
                    userdata.active_strafe_key = 'center'
                    error = self.simple_mover.execute_strafe(angle,
                                                             distance,
                                                             max_velocity = velocity,
                                                             acceleration = accel)
                    rospy.loginfo("EXECUTED STRAFE TO CLEAR: %.1f, error %.3f" %( np.degrees(angle),
                                                                                  np.degrees(error)))
                    #did we exit the move execute because of a pause?
                    if userdata.paused:
                        #wait here for unpause, as long as it takes
                        rospy.loginfo("ROTATE TO CLEAR stopped by pause")
                        while not rospy.is_shutdown() and userdata.paused:
                            rospy.sleep(0.2)
                        #unpaused, try again, changing both goal values to returned error
                        distance = error
                    else:
                        #made it through move without being paused, break out
                        break
                    
                except(TimeoutException):
                        rospy.logwarn("TIMEOUT during simple_motion.")
                        return 'timeout'
                
        
        #blocked, who knows what to do
        else:
            self.active_strafe_key = None
            return 'blocked'
        
        self.active_strafe_key = None
        return 'complete'    

    def stop_if_clear(self):
        if not self.strafes['center']['blocked']:
            rospy.loginfo("ROTATE_TO_CLEAR STOP simple_move on center clear")
            self.clear = True
            return True
    
    def handle_costmap_check(self, costmap_check):
        for strafe_key, blocked in zip(costmap_check.strafe_keys,
                                       costmap_check.blocked):
            self.strafes[strafe_key]['blocked'] = blocked            

#scary class for going to list of points with no obstacle checking,
#good for searching area with strafing
class MoveToPoints(smach.State):
    def __init__(self, tf_listener):
        smach.State.__init__(self,
                             input_keys=['point_list',
                                         'detected_sample',
                                         'odometry_frame',
                                         'stop_on_sample'],
                             output_keys=['simple_move',
                                          'point_list'],
                             outcomes=['next_point',
                                       'complete',
                                       'aborted'])
        
        self.tf_listener = tf_listener
        self.facing_next_point = False
            
    def execute(self, userdata):    
               
        if (len(userdata.point_list) > 0):
            rospy.logdebug("MOVE TO POINTS list: %s" %(userdata.point_list))
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
                                                  distance = distance)
            self.facing_next_point = False
            return 'next_point'                

        else:
            return 'complete'
        
        return 'aborted'

