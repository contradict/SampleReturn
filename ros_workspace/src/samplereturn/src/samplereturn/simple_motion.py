#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from tf import TransformListener
from tf.transformations import euler_from_quaternion

class SimpleMover(object):
  def __init__(self, param_ns=""):
 
    self.tf = TransformListener()

    self.max_velocity = rospy.get_param(param_ns + 'max_velocity', 0.5)
    self.acceleration = rospy.get_param(param_ns + 'acceleration', 0.5)
    self.stop_deceleration = rospy.get_param(param_ns + 'stop_deceleration', 1.5)
    self.time_limit = rospy.get_param(param_ns + 'timeout', 20.0)
    self.loop_rate = rospy.get_param(param_ns + 'loop_rate', 10.0)
    self.steering_angle_epsilon = rospy.get_param(param_ns + 'steering_angle_epsilon', 0.01)

    self.running = False
    self.stop_requested = False

    #publishers and subscribers
    self.publisher = rospy.Publisher("servo_command", Twist)
    rospy.Subscriber("odometry", Odometry, self.odometry_callback, None, 1)
    rospy.Subscriber("platform_joint_state", JointState, self.joint_state_callback, None, 1)

  def odometry_callback(self, msg):
   
    quaternion = np.zeros(4)
    quaternion[0] = msg.pose.pose.orientation.x
    quaternion[1] = msg.pose.pose.orientation.y
    quaternion[2] = msg.pose.pose.orientation.z
    quaternion[3] = msg.pose.pose.orientation.w

    self.current_yaw = euler_from_quaternion(quaternion)[-1]

    self.current_position = msg.pose.pose.position


    self.got_odom = True    

  def distance_traveled(self, goal):
      return np.hypot(self.current_position.x-goal.x,
                      self.current_position.y-goal.y)

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
           rospy.Time.now() < timeout_time and
           not self.stop_requested)
  
  def execute_spin(self, rot, time_limit = None, stop_function = None):
    self.stop_requested = False
    self.running = True

    if time_limit is None:
      time_limit = self.time_limit
    timeout_time = rospy.Time.now()
    timeout_time.secs += time_limit

    rate = rospy.Rate(self.loop_rate)
    
    try:
      pos, quat = self.tf.lookupTransform("/base_link","/stern_suspension",rospy.Time(0))
    except (tf.Exception):
      rospy.logwarn("SIMPLE_MOTION: Failed to get stern_suspension transform")
      return

    #for positive omega, the stern wheel is at -pi/2
    self.target_angle = -np.pi/2 * np.sign(rot)
    
    #limit stern wheel pod to a linear velocity 
    max_stern_vel = self.max_velocity
      
    stopping_yaw = self.max_velocity**2/(2*self.acceleration/np.abs(pos[0]))
    accel_per_loop = self.acceleration/self.loop_rate

    starting_yaw = self.current_yaw
    self.got_odom = False
    self.got_joint_state = False
 
    target_yaw = self.unwind(starting_yaw + rot)
    #start current twist at 0
    current_twist = Twist()
    current_vel = 0

    while self.keep_running(timeout_time):
      # Run at some rate, ~10Hz
      rate.sleep()      
      
      if self.stop_requested or (callable(stop_function) and stop_function()):
        self.stop_requested = True
        accel_per_loop = self.stop_deceleration/self.loop_rate      

      #do not pass here until odom callback fires, 
      #passing means starting_yaw is now initialized
      if not self.got_odom or not self.got_joint_state:
        rospy.logwarn("SIMPLE_MOTION waiting for odom and joint_states")      
        continue
      
      #calculate linear vel of stern wheel pod
      current_vel = np.abs(current_twist.angular.z*np.abs(pos[0]))

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
        current_twist = twist
        #rospy.loginfo("Outgoing twist to set angles: " + str(twist))
        self.publisher.publish(twist)
        continue

      stopping_yaw = current_twist.angular.z**2/(2*self.acceleration/np.abs(pos[0]))
      
      #check if we are at decel point      
      if np.abs(self.unwind(target_yaw - self.current_yaw)) < stopping_yaw:
        break

      #if we are under max_vel keep accelerating      
      elif (current_vel < max_stern_vel):
        twist = current_twist
        twist.angular.z += np.sign(rot)*accel_per_loop/np.abs(pos[0])
        #rospy.loginfo("Current vel: %s, Max vel: %s " % (current_twist.angular.z*np.abs(pos[0]), self.max_velocity))
        #rospy.loginfo("Outgoing twist to accel: " + str(twist))
        self.publisher.publish(twist)
        current_twist = twist
        continue

      #at max_vel and before decel point, keep publishing current twist
      else:
        #rospy.loginfo("Outgoing twist at max_vel: " + str(twist))
        self.publisher.publish(current_twist)

    if self.stop_requested:
      accel_per_loop = self.stop_deceleration/self.loop_rate

    #decel to zero
    for i in range(int(current_vel/accel_per_loop),-1,-1):
      rate.sleep()
      twist = current_twist
      twist.angular.z = np.sign(rot)*accel_per_loop*i/np.abs(pos[0])
      #rospy.loginfo("Outgoing twist to decel: " + str(twist))
      self.publisher.publish(twist)

    self.running = False
    self.stop_requested = False
    
    #check if we are exiting because of timeout
    if rospy.Time.now() > timeout_time:
      return False
    else:
      return True

  def execute_strafe(self, angle, distance, time_limit = None, stop_function = None):
    self.stop_requested = False
    self.running = True

    if time_limit is None:
      time_limit = self.time_limit
    timeout_time = rospy.Time.now()
    timeout_time.secs += time_limit
    rate = rospy.Rate(self.loop_rate)

    starting_position = self.current_position

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

    stopping_distance = self.max_velocity**2/(2*self.acceleration)
    accel_per_loop = self.acceleration/self.loop_rate

    self.got_odom = False
    self.got_joint_state = False
    #empty twist message constructed with all zeroes
    current_twist = Twist()

    while self.keep_running(timeout_time):
      # Run at some rate, ~10Hz
      rate.sleep()

      if self.stop_requested or (callable(stop_function) and stop_function()):
        rospy.loginfo("STOP REQUESTED IN EXECUTE STRAFE")
        self.stop_requested = True
        accel_per_loop = self.stop_deceleration/self.loop_rate
      
      #wait here for odom callback to clear flag, 
      #this means starting_position, is now initialized
      if not self.got_odom or not self.got_joint_state:
        continue

      stopping_distance = (np.sqrt(current_twist.linear.x**2 +
                                        current_twist.linear.y**2)**2 /
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
        current_twist = twist
        continue

      # check if we are must decelerate to stop now
      # distance = (velocity**2)/(2*accel_limit)
      elif (distance-self.distance_traveled(starting_position) < stopping_distance):
        break

      # Accelerate until max_vel reached
      elif (np.sqrt(current_twist.linear.x**2+current_twist.linear.y**2)
                    < self.max_velocity):
        twist = current_twist
        twist.linear.x += accel_per_loop*self.x
        twist.linear.y += accel_per_loop*self.y
        self.publisher.publish(twist)
        current_twist = twist
        continue

      #at max_vel, keep publishing until decel point is hit
      else:
        self.publisher.publish(current_twist)
      
    velocity = np.sqrt(current_twist.linear.x**2 + current_twist.linear.y**2)
    for i in range(int(velocity/accel_per_loop),-1,-1):
      rate.sleep()
      twist = current_twist
      twist.linear.x = accel_per_loop*self.x*i
      twist.linear.y = accel_per_loop*self.y*i
      self.publisher.publish(twist)

    self.running = False
    self.stop_requested = False

    #check if we are exiting because of timeout
    if rospy.Time.now() > timeout_time:
      return False
    else:
      return True

  def is_running(self):
    return self.running
      
  def stop(self):
    self.stop_requested = True


if __name__ == "__main__":
  rospy.init_node('simple_motion')
  simple_motion = SimpleMotion()
  rospy.spin()
