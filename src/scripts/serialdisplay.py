#!/usr/bin/env python
import rospy
import serial

sp = serial.Serial("/dev/ttyUSB0", 115200)
rospy.init_node("led_display")

r = rospy.Rate(10000)

while not rospy.is_shutdown():
    now = rospy.Time.now().to_sec()
    centiseconds = str(int(now*100))[-1]
    #print "%5.3f"%(now-int(now)), centiseconds
    sp.write(centiseconds)
    r.sleep()

sp.close()
