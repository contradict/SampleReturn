#!/usr/bin/env python

import roslib; roslib.load_manifest('samplereturn')
import rospy

from platform_motion_msgs.msg import GPIO
from std_msgs.msg import Bool

class LightController(object):
    _pin_mask = 7 #mask that defines the light pins
    _all_on = 0 #bit states for all lights on
    _all_off = 7 #bit states for all lights off
     
    def __init__(self):
        rospy.Subscriber("pause_state", Bool, self.handle_pause_state)
        rospy.Subscriber("search_lights", Bool, self.handle_search_lights)
        self.pub = rospy.Publisher('gpio_write', GPIO)
        self.blink = False
        self.light_state = self._all_on
        self.search_light_state = False
        self.cycle = rospy.Timer(rospy.Duration(1.0), self.set_lights)

    def set_lights(self, event): #rospy.Timer passes in an event object to this cb
        if self.blink:
            self.light_state = self._all_on if (self.light_state == self._all_off)\
                               else self._all_off
        else:
            self.light_state = self._all_on
        pin_mask = 0x0f
        new_pin_states = self.light_state | ((not self.search_light_state)<<3)
        self.pub.publish(GPIO(servo_id=1,
                              pin_mask=pin_mask,
                              new_pin_states=new_pin_states))
        
    def handle_pause_state(self, msg):
        self.blink = not msg.data

    def handle_search_lights(self, msg):
        self.search_lights(msg.data)
        
    def search_lights(self, on):
        self.search_light_state = on
        mask=8
        pin_states = 0 if on else mask
        self.pub.publish(GPIO(servo_id = 1,
                         pin_mask = mask,
                         new_pin_states = pin_states))
        
def main():
    rospy.init_node('lights')
    b = LightController()
    rospy.spin()

if __name__=="__main__":
    main()
