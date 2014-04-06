import rospy

import threading
import traceback
import collections

import smach


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
