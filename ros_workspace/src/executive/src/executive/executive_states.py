
import threading
import traceback
import collections

import rospy
import actionlib
import tf
import smach

import std_msgs.msg as std_msg
import geometry_msgs.msg as geometry_msg
import actionlib_msgs.msg as action_msg
import move_base_msgs.msg as move_base_msg
import samplereturn_msgs.msg as samplereturn_msg

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
                             outcomes=['next', 'timeout'],
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
                             outcomes=['complete', 'timeout', 'sample_detected', 'preempted','aborted'],
                             input_keys=['target_pose', 'velocity', 'pursue_samples', 'detected_sample'],
                             output_keys=['actual_point'])
        
        self.move_client = move_client

        self.listener = listener
        self.sample_detected = False
                
    def execute(self, ud):
        
        start_pose = util.get_current_robot_pose(self.listener)
        target_pose = ud.target_pose
        velocity = ud.velocity
        
        distance = util.point_distance_2d(start_pose.pose.position,
                                          target_pose.pose.position)
        
        goal = move_base_msg.MoveBaseGoal()
        goal.target_pose=target_pose
        self.move_client.send_goal(goal)
        while True:
            rospy.sleep(0.1)
            move_state = self.move_client.get_state()
            if move_state not in util.actionlib_working_states:
                break
            if (ud.detected_sample is not None) and ud.pursue_samples:
                return 'sample_detected'

        if move_state == action_msg.GoalStatus.SUCCEEDED:
            return 'complete'
        else:
            return 'aborted'

#pursues a detected point, changing the move_base goal if the detection point
#changes too much.  Detection and movement all specified in /map.
#exits once within min_pursuit_distance, and sets target_pose to the current
#detection point with current robot orientation
class PursueDetectedPoint(smach.State):
    def __init__(self,
                 announcer,
                 move_client,
                 listener,
                 within_min_msg = None):
        smach.State.__init__(self,
                             outcomes=['min_distance', 'point_lost', 'preempted', 'aborted'],
                             input_keys=['target_point',
                                         'min_pursuit_distance',
                                         'max_pursuit_error',
                                         'max_point_lost_time'],
                             output_keys=['target_point',
                                          'target_pose',
                                          'velocity',
                                          'pursue_samples'])
    
        self.announcer = announcer
        self.move_client = move_client
        self.listener = listener
        self.within_min_msg = within_min_msg

    def execute(self, userdata):
        
        move_goal = move_base_msg.MoveBaseGoal()
        header = std_msg.Header(0, rospy.Time(0), '/map')
        start_pose = util.get_current_robot_pose(self.listener)
        rospy.loginfo("PURSUIT start pose: " + str(start_pose))
        point_on_map = self.listener.transformPoint('/map', userdata.target_point)
        
        goal_pose = geometry_msg.Pose()
        goal_pose.position = point_on_map.point
        goal_pose.orientation = start_pose.pose.orientation
        move_goal.target_pose = geometry_msg.PoseStamped(header, goal_pose) 
        self.move_client.send_goal(move_goal)
        
        rospy.loginfo("PURSUIT initial target point: %s", point_on_map)

        last_point_detection = rospy.get_time()

        while True:
            current_pose= util.get_current_robot_pose(self.listener)
            move_state = self.move_client.get_state()
            if move_state not in util.actionlib_working_states:
                return 'aborted'
            
            point_distance = util.pose_distance_2d(current_pose, move_goal.target_pose)
            rospy.loginfo("PURSUIT distance: " + str(point_distance))
            if point_distance < userdata.min_pursuit_distance:
                userdata.target_pose = move_goal.target_pose
                if self.within_min_msg is not None:
                    self.announcer.say(self.within_min_msg)
                return 'min_distance'
            
            if userdata.target_point is not None:
                last_point_detection = rospy.get_time()
                point_on_map = self.listener.transformPoint('/map', userdata.target_point)
                
                current_point = move_goal.target_pose.pose.position
                sample_distance_delta = util.point_distance_2d(current_point, point_on_map.point)
                if  sample_distance_delta > userdata.max_pursuit_error:
                    rospy.loginfo("PURSUE point updated: %s", point_on_map)
                    move_goal.target_pose.pose.position = point_on_map.point
                    self.move_client.send_goal(move_goal)
            
            if (rospy.get_time() - last_point_detection) > userdata.max_point_lost_time:
                return 'point_lost'
            
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
                            
            userdata.target_point = None       
            rospy.sleep(0.2)
        

