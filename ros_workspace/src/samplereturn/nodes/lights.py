#!/usr/bin/env python

import roslib; roslib.load_manifest('samplereturn')
import rospy

from platform_motion.msg import GPIO

class blinker(object):
    def __init__(self):
        self.pub = rospy.Publisher('gpio_write', GPIO)
        self.light_state = 1
        self.cycle = rospy.Timer(rospy.Duration(0.1), self.shift_lights)

    def shift_lights(self, event):
        self.pub.publish(GPIO(servo_id=5,pin_mask=7,new_pin_states=self.light_state))
        self.light_state = ((self.light_state*2)&0x7) + ((self.light_state&0x4)/4)

def main():
    rospy.init_node('blinker')
    b = blinker()
    rospy.spin()

if __name__=="__main__":
    main()
