#!/usr/bin/env python

import roslib; roslib.load_manifest('samplereturn')
import rospy

from platform_motion_msgs.msg import GPIO
from std_msgs.msg import Bool

class LightController(object):
     
    def __init__(self):
        self.servo_id = rospy.get_param("servo_id", 1)
        self.pause_lights = rospy.get_param("pause_lights", 0x01)
        self.search_lights = rospy.get_param("search_lights", 0x08)
        rospy.Subscriber("pause_state", Bool, self.handle_pause_state)
        rospy.Subscriber("search_lights", Bool, self.handle_search_lights)
        self.pub = rospy.Publisher('gpio_write', GPIO, queue_size=1)
        self.blink = False
        self.pause_light_state = False
        self.search_light_state = False
        self.cycle = rospy.Timer(rospy.Duration(1.0), self.set_lights)
        rospy.on_shutdown( self.lights_off )

    def set_lights(self, event): #rospy.Timer passes in an event object to this cb
        if self.blink:
            self.pause_light_state ^= True
        else:
            self.pause_light_state = True
        pin_mask = self.pause_lights | self.search_lights
        new_pin_states = (not self.pause_light_state and self.pause_lights or 0) | \
                         (not self.search_light_state and self.search_lights or 0)
        self.pub.publish(GPIO(servo_id=self.servo_id,
                              pin_mask=pin_mask,
                              new_pin_states=new_pin_states))

    def lights_off(self):
        self.cycle.shutdown()
        self.pub.publish(GPIO(servo_id=self.servo_id,
                              pin_mask=self.search_lights|self.pause_lights,
                              new_pin_states=self.search_lights|self.pause_lights))
        rospy.sleep(0.1)

    def handle_pause_state(self, msg):
        self.blink = not msg.data

    def handle_search_lights(self, msg):
        self.search_light_state = msg.data
        mask=self.search_lights
        pin_states = 0 if self.search_light_state else mask
        self.pub.publish(GPIO(servo_id = 1,
                         pin_mask = mask,
                         new_pin_states = pin_states))
        
def main():
    rospy.init_node('lights')
    b = LightController()
    rospy.spin()

if __name__=="__main__":
    main()
