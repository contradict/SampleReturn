#!/usr/bin/env python

import roslib; roslib.load_manifest('samplereturn')
import rospy

from platform_motion.msg import GPIO
from std_msgs.msg import Bool

class LightController(object):
    _pin_mask = 7 #mask that defines the light pins
    _all_on = 0 #bit states for all lights on
    _all_off = 7 #bit states for all lights off
     
    def __init__(self):
        rospy.Subscriber("/pause_state", Bool, self.pause_state_update)
        self.pub = rospy.Publisher('gpio_write', GPIO)
        self.blink = True
        self.light_state = self._all_on
        self.cycle = rospy.Timer(rospy.Duration(1.0), self.set_lights)

    def set_lights(self, event): #rospy.Timer passes in an event object to this cb
        if self.blink:
            self.light_state = self._all_on if (self.light_state == self._all_off)\
                               else self._all_off
        else:
            self.light_state = self._all_on
        self.pub.publish(GPIO(servo_id=1, pin_mask=7, new_pin_states=self.light_state))
        
    def pause_state_update(self, pause_state_msg):
        self.blink = not pause_state_msg.data
        
def main():
    rospy.init_node('lights')
    b = LightController()
    rospy.spin()

if __name__=="__main__":
    main()
