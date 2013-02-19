#!/usr/bin/python
import curses
import sys
import math
import rospy
from geometry_msgs.msg import Twist

class SinusoidTest(object):
    def __init__(self):
        rospy.init_node('sinusoid_test')
        self.twist_pub=rospy.Publisher("/twist", Twist)
        self.frequency=0.5;
        self.velocity=0.01;
        self.saved_velocity=0.01;
        self.theta_magnitude=0.5;
        self.phase = 0;
        self.time = None;
        self.stdscr = curses.initscr()
        curses.cbreak()
        curses.echo(0)
        self.stdscr.keypad(1)
        self.stdscr.nodelay(1)
        rospy.Timer(rospy.Duration(0.050), self.tick)

    def tick(self, event):
        if self.time is None:
            self.phase = 0
        else:
            self.advance_phase((event.current_real - self.time).to_sec())
        self.time = event.current_real
        self.compute_velocity()
        self.send_twist(self.phase)
        pc=self.stdscr.getch()
        if pc != -1:
            self.process_char(pc)
        self.update_display()

    def advance_phase(self, dt):
        self.phase += dt*self.frequency
        while self.phase>2*math.pi:
            self.phase -= 2*math.pi

    def compute_velocity(self):
        self.theta=self.theta_magnitude*math.sin(self.phase)
        self.vx=self.velocity*math.cos(self.theta)
        self.vy=self.velocity*math.sin(self.theta)
        self.vt=0

    def send_twist(self, phase):
        tw=Twist()
        tw.linear.x=self.vx
        tw.linear.y=self.vy
        tw.linear.z=0
        tw.angular.x=0
        tw.angular.y=0
        tw.angular.z=self.vt
        self.twist_pub.publish(tw)

    def send_zero(self):
        tw=Twist()
        tw.linear.x=0
        tw.linear.y=0
        tw.linear.z=0
        tw.angular.x=0
        tw.angular.y=0
        tw.angular.z=0
        self.twist_pub.publish(tw)

    def process_char(self, pc):
        c=chr(pc)
        self.stdscr.addstr(2, 0, "%s"%c)
        if c==' ':
            self.saved_velocity=self.velocity
            self.velocity = 0.0
            self.send_zero()
        elif c=='g':
            self.velocity = self.saved_velocity
        elif c=='w':
            self.velocity += 0.01
        elif c=='s':
            self.velocity -= 0.01
        elif c=='a':
            self.theta_magnitude -= 0.01
        elif c=='d':
            self.theta_magnitude += 0.01
        elif c=='r':
            self.frequency += 0.1
        elif c=='f':
            self.frequency -= 0.1
        elif c=='q' or pc==27:
            self.send_zero()
            self.cleanup()
            rospy.signal_shutdown("operator exit")

    def update_display(self):
        self.stdscr.addstr(0,0, "%04.3f %04.3f"%(self.velocity, self.theta_magnitude))
        self.stdscr.addstr(1,0, "%04.3f %04.3f %04.3f"%(self.vx, self.vy, self.vt))

    def cleanup(self):
        try:
            curses.nocbreak()
            self.stdscr.keypad(0)
            self.stdscr.nodelay(0)
            curses.echo()
            curses.endwin()
        except:
            pass

    def __del__(self):
        self.cleanup()

if __name__=="__main__":
    SinusoidTest()
    rospy.spin()
