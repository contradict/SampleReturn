#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Strafe(object):
  def __init__(self, param_string=""):
    rospy.Subscriber("/motion/odometry", Odometry, self.odometry_callback, None, 1)
    rospy.Subscriber("/motion/platform_joint_state", JointState, self.joint_state_callback, None, 1)
    self.publisher = rospy.Publisher("strafe_command", Twist)

    self.loop_rate = rospy.get_param('~loop_rate',10.0)
    self.max_velocity = rospy.get_param('~max_velocity',2.0)
    self.max_acceleration = rospy.get_param('~max_acceleration',1.0)
    self.time_limit = rospy.get_param('~time_limit',20.0)
    self.wheel_pos_epsilon = rospy.get_param('~wheel_pos_epsilon',0.01)

    self.accel_per_loop = self.max_acceleration/self.loop_rate

    self.current_twist = None
    self.starting_position = None

  def odometry_callback(self, msg):
    if self.starting_position == None:
      self.starting_position = msg.pose.pose.position

    self.current_position = msg.pose.pose.position

    self.distance_traveled = np.sqrt((self.current_position.x-self.starting_position.x)**2 +
                                     (self.current_position.y-self.starting_position.y)**2)
    self.got_odom = True

  def joint_state_callback(self, msg):
    pos_dict = dict(zip(msg.name,msg.position))
    vel_dict = dict(zip(msg.name,msg.velocity))
    self.stern_pos      = pos_dict["stern_steering_joint"]
    self.port_pos       = pos_dict["port_steering_joint"]
    self.starboard_pos  = pos_dict["starboard_steering_joint"]
    self.stern_vel      = vel_dict["stern_steering_joint"]
    self.port_vel       = vel_dict["port_steering_joint"]
    self.starboard_vel  = vel_dict["starboard_steering_joint"]

    self.got_joint_state = True

  def execute(self, angle, distance, time_limit=20.0):
    shutdown_time = rospy.Time.now()
    shutdown_time.secs += time_limit

    rate = rospy.Rate(self.loop_rate)

    if (-np.pi/2<=angle<=np.pi/2):
      self.target_angle = angle
    elif (np.pi/2<angle<np.pi*1.5):
      self.target_angle = angle - np.pi
    elif (np.pi*1.5<angle<2*np.pi):
      self.target_angle = angle - 2*np.pi

    self.target_x = np.cos(self.target_angle)
    self.target_y = np.sin(self.target_angle)

    self.stopping_distance = self.max_velocity**2/(2*self.max_acceleration)

    self.shutdown = False
    self.got_odom = False
    self.got_joint_state = False

    while not self.shutdown and rospy.Time.now()<shutdown_time:
      if not self.got_odom or not self.got_joint_state:
        continue

      # Run at some rate, ~10Hz
      rate.sleep()
      # Issue slow twists until wheels are pointed at angle
      if (np.abs(self.stern_pos-self.target_angle)<self.wheel_pos_epsilon and
          np.abs(self.port_pos-self.target_angle)<self.wheel_pos_epsilon and
          np.abs(self.starboard_pos-self.target_angle)<self.wheel_pos_epsilon):
        twist = Twist()
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        twist.linear.x = self.target_x*0.001
        twist.linear.y = self.target_y*0.001
        twist.linear.z = 0.0
        self.current_twist = twist
        self.publisher.publish(twist)
        continue

      # Accelerate until max_vel reached
      elif (np.sqrt(self.current_twist.linear.x**2+self.current_twist.linear.y**2) < self.max_velocity):
        twist = self.current_twist
        twist.linear.x += self.accel_per_loop*self.target_x
        twist.linear.y += self.accel_per_loop*self.target_y
        self.publisher.publish(twist)
        self.current_twist = twist
        continue

      # Decelerate to stop at distance (velocity**2)/(2*accel_limit)
      elif (distance-self.distance_traveled) < self.stopping_distance:
        twist = self.current_twist
        twist.linear.x -= self.accel_per_loop*self.target_y
        twist.linear.y -= self.accel_per_loop*self.target_y
        self.publisher.publish(twist)
        self.current_twist = twist

      else:
        self.publisher.publish(self.current_twist)

  def shutdown(self):
    self.shutdown = True
    rate = rospy.Rate(self.loop_rate)
    while np.abs(self.current_twist.linear.x+self.current_twist.linear.y) > self.accel_per_loop:
      twist = self.current_twist
      twist.linear.x -= self.accel_per_loop*self.target_x
      twist.linear.y -= self.accel_per_loop*self.target_y
      self.current_twist = twist
      self.publisher.publish(twist)
      rate.sleep()

    twist = Twist()
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0
    self.publisher.publish(twist)

if __name__ == "__main__":
  rospy.init_node('strafe')
  strafe = Strafe()
  rospy.spin()
