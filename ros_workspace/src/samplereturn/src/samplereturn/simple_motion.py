#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import TransformListener
from tf.transformations import euler_from_quaternion

class SimpleMover(object):
  def __init__(self, param_ns=""):

    self.publisher = rospy.Publisher("servo_command", Twist)
    self.tf = TransformListener()

    self.max_velocity = rospy.get_param(param_ns + 'max_velocity', 0.5)
    self.acceleration = rospy.get_param(param_ns + 'acceleration', 0.5)
    self.stop_deceleration = rospy.get_param(param_ns + 'stop_deceleration', 1.5)
    self.time_limit = rospy.get_param(param_ns + 'timeout', 20.0)
    self.loop_rate = rospy.get_param(param_ns + 'loop_rate', 10.0)
    self.steering_angle_epsilon = rospy.get_param(param_ns + 'steering_angle_epsilon', 0.01)

    self.accel_per_loop = self.acceleration/self.loop_rate

    self.current_twist = None
    self.starting_position = None
    self.starting_yaw = None
    self.running = False
    self.stop_requested = False

    rospy.Subscriber("/motion/odometry", Odometry, self.odometry_callback, None, 1)
    rospy.Subscriber("/motion/platform_joint_state", JointState, self.joint_state_callback, None, 1)

  def odometry_callback(self, msg):
   
    quaternion = np.zeros(4)
    quaternion[0] = msg.pose.pose.orientation.x
    quaternion[1] = msg.pose.pose.orientation.y
    quaternion[2] = msg.pose.pose.orientation.z
    quaternion[3] = msg.pose.pose.orientation.w

    if self.starting_yaw is None:
      self.starting_yaw = euler_from_quaternion(quaternion)[-1]

    if self.starting_position is None:
      self.starting_position = msg.pose.pose.position

    self.current_yaw = euler_from_quaternion(quaternion)[-1]

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

  def unwind(self, ang):
    if ang > np.pi:
      ang -= 2*np.pi
    elif ang < np.pi:
      ang += 2*np.pi
    return ang

  def unwind_90(self, ang):
      if -np.pi/2 <= ang <= np.pi/2:
          return ang
      elif ang > np.pi/2:
          ang -= np.pi
          return ang
      elif ang < -np.pi/2:
          ang += np.pi
          return ang

  #method to check if the movement loop should keep running, or bail out
  def keep_running(self, timeout_time):
    return (not rospy.is_shutdown() and 
           rospy.time.now() < timeout_time and
           not self.stop_requested)
  
  def execute_spin(self, rot, time_limit=10.0):
    
    self.running = True
    
    try:
      pos, quat = self.tf.lookupTransform("/base_link","/stern_suspension",rospy.Time(0))
    except (tf.Exception):
      rospy.logwarn("SIMPLE_MOTION: Failed to get stern_suspension transform")
      return

    timeout_time = rospy.Time.now()
    timeout_time.secs += time_limit

    rate = rospy.Rate(self.loop_rate)

    #for positive omega, the stern wheel is at -pi/2
    self.target_angle = -np.pi/2 * np.sign(rot)
    
    #limit stern wheel pod to a linear velocity 
    max_stern_vel = self.max_velocity
      
    self.stopping_yaw = self.max_velocity**2/(2*self.acceleration/np.abs(pos[0]))

    self.starting_yaw = None
    self.got_odom = False
    self.got_joint_state = False
 
    while self.keep_running(timeout_time):
      #wait here for odom callback to clear flag, 
      #this means starting_yaw is now initialized
      rate.sleep()
      if self.got_odom and self.got_joint_state:
        break
      rospy.logwarn("SIMPLE_MOTION waiting for odom and joint_states")
      
    self.target_yaw = self.unwind(self.starting_yaw + rot)
    twist = Twist()
    self.current_twist = twist
    current_vel = 0

    while self.keep_running(timeout_time):
      # Run at some rate, ~10Hz
      rate.sleep()      
      #calculate linear vel of stern wheel pod
      current_vel = np.abs(self.current_twist.angular.z*np.abs(pos[0]))

      #print "target_angle: %s" % (str(self.target_angle))
      #print "stern_pos: %s, port_pos: %s, star_pos: %s" % (str(self.stern_pos), str(self.port_pos), str(self.starboard_pos))
      #print "epsilon: %s" % (str(self.steering_angle_epsilon))
      #print "stern_eps: %s" % (str(np.abs(self.stern_pos-self.target_angle)))

      #wait until correct steering angles are achieved
      if (np.abs(self.stern_pos-self.target_angle)>self.steering_angle_epsilon or
          np.abs(self.port_pos)>self.steering_angle_epsilon or
          np.abs(self.starboard_pos)>self.steering_angle_epsilon):
        twist = Twist()
        twist.angular.z = 0.001*np.sign(rot)
        self.current_twist = twist
        #rospy.loginfo("Outgoing twist to set angles: " + str(twist))
        self.publisher.publish(twist)
        continue

      self.stopping_yaw = self.current_twist.angular.z**2/(2*self.acceleration/np.abs(pos[0]))
      
      #check if we are at decel point      
      if np.abs(self.unwind(self.target_yaw - self.current_yaw)) < self.stopping_yaw:
        break

      #if we are under max_vel keep accelerating      
      elif (current_vel < max_stern_vel):
        twist = self.current_twist
        twist.angular.z += np.sign(rot)*self.accel_per_loop/np.abs(pos[0])
        #rospy.loginfo("Current vel: %s, Max vel: %s " % (self.current_twist.angular.z*np.abs(pos[0]), self.max_velocity))
        #rospy.loginfo("Outgoing twist to accel: " + str(twist))
        self.publisher.publish(twist)
        self.current_twist = twist
        continue

      #at max_vel and before decel point, keep publishing current twist
      else:
        #rospy.loginfo("Outgoing twist at max_vel: " + str(twist))
        self.publisher.publish(self.current_twist)

    #decel to zero
    for i in range(int(current_vel/self.accel_per_loop),-1,-1):
      rate.sleep()
      twist = self.current_twist
      twist.angular.z = np.sign(rot)*self.accel_per_loop*i/np.abs(pos[0])
      #rospy.loginfo("Outgoing twist to decel: " + str(twist))
      self.publisher.publish(twist)

    self.running = False
    self.stop_requested = False

  def execute_strafe(self, angle, distance, time_limit=20.0):
    self.starting_position = None
    self.current_twist = None

    timeout_time = rospy.Time.now()
    timeout_time.secs += time_limit

    rate = rospy.Rate(self.loop_rate)

    if (-np.pi/2<=angle<=np.pi/2):
      self.target_angle = angle
    elif (np.pi/2 < angle):
      self.target_angle = angle - np.pi
    elif (angle < -np.pi/2):
      self.target_angle = angle + np.pi

    self.target_x = np.cos(self.target_angle)
    self.target_y = np.sin(self.target_angle)

    self.x = np.cos(angle)
    self.y = np.sin(angle)

    self.stopping_distance = self.max_velocity**2/(2*self.acceleration)

    self.got_odom = False
    self.got_joint_state = False
    #empty twist message constructed with all zeroes
    twist = Twist()
    self.current_twist = twist

    while self.keep_runing(timeout_time):
      # Run at some rate, ~10Hz
      rate.sleep()
      
      #wait here for odom callback to clear flag, 
      #this means starting_position, is now initialized
      if not self.got_odom or not self.got_joint_state:
        continue

      self.stopping_distance = (np.sqrt(self.current_twist.linear.x**2 +
                                        self.current_twist.linear.y**2)**2 /
                                        (2*self.acceleration))

      #very useful debug messages
      #print "target_angle: %s" % (str(self.target_angle))
      #print("stern_pos: %s, port_pos: %s, star_pos: %s" % (str(self.stern_pos), str(self.port_pos), str(self.starboard_pos)))

      # Issue slow twists until wheels are pointed at angle
      if (np.abs(self.stern_pos-self.target_angle)>self.steering_angle_epsilon or
          np.abs(self.port_pos-self.target_angle)>self.steering_angle_epsilon or
          np.abs(self.starboard_pos-self.target_angle)>self.steering_angle_epsilon):
        twist = Twist()
        twist.linear.x = self.x*0.001
        twist.linear.y = self.y*0.001
        self.publisher.publish(twist)
        self.current_twist = twist
        continue

      # check if we are must decelerate to stop now
      # distance = (velocity**2)/(2*accel_limit)
      elif (distance-self.distance_traveled) < self.stopping_distance:
        break

      # Accelerate until max_vel reached
      elif (np.sqrt(self.current_twist.linear.x**2+self.current_twist.linear.y**2)
                    < self.max_velocity):
        twist = self.current_twist
        twist.linear.x += self.accel_per_loop*self.x
        twist.linear.y += self.accel_per_loop*self.y
        self.publisher.publish(twist)
        self.current_twist = twist
        continue

      #at max_vel, keep publishing until decel point is hit
      else:
        self.publisher.publish(self.current_twist)

    velocity = np.sqrt(self.current_twist.linear.x**2 + self.current_twist.linear.y**2)
    for i in range(int(velocity/self.accel_per_loop),-1,-1):
      rate.sleep()
      twist = self.current_twist
      twist.linear.x = self.accel_per_loop*self.x*i
      twist.linear.y = self.accel_per_loop*self.y*i
      self.publisher.publish(twist)

    self.running = False
    self.stop_requested = False

  def is_running(self):
    return self.running
      
  def stop(self):
    self.stop_requested = True


if __name__ == "__main__":
  rospy.init_node('simple_motion')
  simple_motion = SimpleMotion()
  rospy.spin()
