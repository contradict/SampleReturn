#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from tf import TransformListener
from tf.transformations import euler_from_quaternion
import samplereturn.util as util
from samplereturn_msgs.srv import SimpleMotionResponse, SimpleMotion

class SimpleMover(object):
  # TODO: Actually calculate something for this
  steering_timeout = 5
  def __init__(self, param_ns="", listener=None, stop_function=None):
    if listener is not None:
      self.tf = listener
    else:
      self.tf = TransformListener()
      rospy.sleep(2.0)
      
    self.param_ns = param_ns
      
    self.default_stop_function = stop_function

    self.max_velocity = rospy.get_param(param_ns + 'max_velocity', 0.5)
    self.acceleration = rospy.get_param(param_ns + 'acceleration', 0.5)
    self.deceleration = self.acceleration
    self.stop_deceleration = rospy.get_param(param_ns + 'stop_deceleration', 1.5)
    self.loop_rate = rospy.get_param(param_ns + 'loop_rate', 10.0)
    self.steering_angle_epsilon = rospy.get_param(param_ns + 'steering_angle_epsilon', 0.01)
    self.velocity_epsilon = rospy.get_param(param_ns + 'velocity_epsilon', 0.001)

    self.running = False
    self.stop_requested = False
    
    #this is a hack to allow changes in strafe direction
    self.strafe_angle = None

    #get stern wheel transform
    try:
      self.tf.waitForTransform('base_link', 'stern_suspension',
                               rospy.Time(0), rospy.Duration(5.0))
      pos, quat = self.tf.lookupTransform("base_link","stern_suspension",rospy.Time(0))
      self.stern_offset = np.abs(pos[0])
    except (tf.Exception):
      rospy.logwarn("SIMPLE_MOTION: Failed to get stern_suspension transform")
      return

    #publishers and subscribers
    self.publisher = rospy.Publisher("twist", Twist, queue_size=1)
    rospy.Subscriber("odometry", Odometry, self.odometry_callback, None, 1)
    rospy.Subscriber("platform_joint_state", JointState, self.joint_state_callback, None, 1)

  def odometry_callback(self, msg):
    """
    Called to store present robot position and yaw
    """
    quaternion = np.zeros(4)
    quaternion[0] = msg.pose.pose.orientation.x
    quaternion[1] = msg.pose.pose.orientation.y
    quaternion[2] = msg.pose.pose.orientation.z
    quaternion[3] = msg.pose.pose.orientation.w

    self.current_yaw = euler_from_quaternion(quaternion)[-1]

    self.current_position = np.r_[msg.pose.pose.position.x,
                                  msg.pose.pose.position.y,
                                  msg.pose.pose.position.z]
            
    self.current_omega = msg.twist.twist.angular.z
    self.current_speed = np.hypot( msg.twist.twist.linear.x,
                                   msg.twist.twist.linear.y)
    self.current_angle = np.arctan2(msg.twist.twist.linear.y,
                                    msg.twist.twist.linear.x)

  def joint_state_callback(self, msg):
    """
    Remember current robot steering angles
    """
    pos_dict = dict(zip(msg.name,msg.position))
    vel_dict = dict(zip(msg.name,msg.velocity))
    self.stern_pos      = pos_dict["stern_steering_joint"]
    self.port_pos       = pos_dict["port_steering_joint"]
    self.starboard_pos  = pos_dict["starboard_steering_joint"]
    self.stern_vel      = vel_dict["stern_steering_joint"]
    self.port_vel       = vel_dict["port_steering_joint"]
    self.starboard_vel  = vel_dict["starboard_steering_joint"]

  def spin_publisher(self, velocity, sign):
    twist = Twist()
    twist.angular.z = sign*velocity/self.stern_offset
    self.publisher.publish(twist)

  #set the strafe angle, and the publisher should just send the robot off at a new angle  
  def set_strafe_angle(self, new_angle):
    self.strafe_angle = new_angle

  def strafe_publisher(self, velocity):
    twist = Twist()
    twist.linear.x = velocity*np.cos(self.strafe_angle)
    twist.linear.y = velocity*np.sin(self.strafe_angle)
    self.publisher.publish(twist)

  def execute_spin(self, rotation, max_velocity=None, acceleration=None, stop_function=None):
    return util.unwind(
            self.execute(
            lambda start_yaw=self.current_yaw:(self.current_yaw-start_yaw-rotation)*self.stern_offset,
            dict(stern=-np.pi/2*np.sign(rotation),
                 port=0.0,
                 starboard=0.0),
            lambda v, sign=np.sign(rotation) : self.spin_publisher(v, sign),
            max_velocity,
            acceleration,
            stop_function)/self.stern_offset)

  def execute_strafe(self, angle, distance, max_velocity=None, acceleration=None, stop_function=None):
    angle = util.unwind(angle)
    if (-np.pi/2<=angle<=np.pi/2):
      target_angle = angle
    elif (np.pi/2 < angle):
      target_angle = angle - np.pi
    elif (angle < -np.pi/2):
      target_angle = angle + np.pi
    self.strafe_angle = target_angle 
    return self.execute(
            lambda start_position=self.current_position: np.abs(distance-np.linalg.norm(start_position-self.current_position)),
            dict(stern=target_angle,
                port=target_angle,
                starboard=target_angle),
            lambda v : self.strafe_publisher(v),
            max_velocity,
            acceleration,
            stop_function)
  
  #use with caution, strafes at constant velocity until external stop is called
  def execute_continuous_strafe(self, angle, max_velocity=None, acceleration=None, stop_function=None):
    angle = util.unwind(angle)
    if (-np.pi/2<=angle<=np.pi/2):
      target_angle = angle
    elif (np.pi/2 < angle):
      target_angle = angle - np.pi
    elif (angle < -np.pi/2):
      target_angle = angle + np.pi
    self.strafe_angle = target_angle
    return self.execute(
            lambda: np.inf ,
            dict(stern=target_angle,
                port=target_angle,
                starboard=target_angle),
            lambda v : self.strafe_publisher(v),
            max_velocity,
            acceleration,
            stop_function)  

  def execute(self, error, target, publisher, max_velocity=None, acceleration=None, stop_function=None):
    rospy.logdebug("SIMPLE_MOTION: {}, starting execute".format(self.param_ns))
    
    rate = rospy.Rate(self.loop_rate)

    #load defaults if no kwargs present
    if max_velocity is None:
      max_velocity = self.max_velocity
    if acceleration is None:
      acceleration = self.acceleration
    self.deceleration = acceleration

    if not callable(stop_function): 
      stop_function = self.default_stop_function

    # if one step of accel overshoots half the distance, give up
    if np.abs(error()/2.) < 0.5*acceleration/self.loop_rate**2:
      return error()
    
    #prepare to start main loop
    self.stop_requested = False
    self.running = False
    steering_timeout_time = rospy.Time.now() + rospy.Duration(self.steering_timeout)
    velocity = lambda:self.velocity_epsilon
    v = velocity()
    while ((not self.should_stop(stop_function)) and (v != 0)):
      # Run at some rate, ~10Hz
      rate.sleep()
      #wait until correct steering angles are achieved
      if ((np.abs(self.stern_pos-target['stern'])>self.steering_angle_epsilon or
           np.abs(self.port_pos-target['port'])>self.steering_angle_epsilon or
           np.abs(self.starboard_pos-target['starboard'])>self.steering_angle_epsilon) and
           not self.running):
        rospy.logdebug("SIMPLE_MOTION: {}, Turning to target angles: {:f} {:f} {:f}".format(
                self.param_ns,
                np.abs(self.stern_pos-target['stern']),
                np.abs(self.port_pos-target['port']),
                np.abs(self.starboard_pos-target['starboard'])))
        if (rospy.Time.now()>steering_timeout_time):
          publisher(0)
          raise TimeoutException('Steering move failed to complete before timeout: {!s}'.format(steering_timeout_time))
      #we have achieved proper steering angle, but not started the motion yet
      #get the velocity function with total_distance as the current error
      elif not self.running:
        start_time = rospy.Time.now()
        velocity = lambda start_time = start_time, d = np.abs(error()) : self.velocity_of_time( d, (rospy.Time.now() - start_time).to_sec(), acceleration, max_velocity)
        self.running = True
      v = velocity()
      publisher(v)
   
    #decel to zero (this is necessary after a stop is requested)
    while v > 0:
      #if an estop was requested, deceleration will be stop_deceleration
      v = np.max((v - (self.deceleration/self.loop_rate), 0))
      publisher(v)
      rate.sleep()

    self.running = False
    self.stop_requested = False

    rospy.logdebug("SIMPLE_MOTION: {}, exiting execute".format(self.param_ns))

    return error()

  def should_stop(self, stop_function):
    external_stop = (callable(stop_function) and stop_function())
    if external_stop:
        self.stop_requested = True
    return self.stop_requested


  def velocity_of_time(self, total_distance, t, acceleration, max_velocity):
    #if total_distance is np.inf, set it equal to accel ramps distance +
    #distance traveled at max_velocity.  This make this loop get to max_v and go forever!    
    if np.isinf(total_distance):
      total_distance = max_velocity**2/acceleration + t*max_velocity
    if( max_velocity**2/2./acceleration > total_distance/2. ):
      # cannot accel to max velocity, compute peak velocity
      vmax = np.sqrt(2*(total_distance/2.)*acceleration)
      # and time of max velocity
      taccel = vmax/acceleration
      if t < taccel:
        vel = t*acceleration
      elif t < 2*taccel:
        vel = vmax-(t-taccel)*acceleration
      else:
        vel = 0
    else:
      taccel = max_velocity/acceleration
      daccel = max_velocity**2/2./acceleration
      dconst = total_distance-2*daccel
      tconst = dconst/max_velocity
      if t < taccel:
        vel = t*acceleration
      elif t < taccel+tconst:
        vel = max_velocity
      elif t < 2*taccel+tconst:
        vel = max_velocity - acceleration*(t-taccel-tconst)
      else:
        vel = 0
    return vel

  def decel_at_current(self, acceleration):
    timeout = np.max((self.current_speed/acceleration,
                      self.current_omega*self.stern_offset/acceleration))
    if timeout > 1./self.loop_rate:
      timeout_time = rospy.Time.now() + rospy.Duration( timeout*1.5 )
      tolerance = self.acceleration*self.loop_rate
      self.stop_requested = False
      accel_per_loop = acceleration/self.loop_rate
      angle = self.current_angle
      while ((rospy.Time().now() < timeout_time) and
             (not self.stop_requested) and
             (self.current_speed>tolerance)):
          if (self.current_omega*self.stern_offset>tolerance):
            raise TimeoutException("Could not stop before adjusting steering")
          omega = np.sign(self.current_omega)*np.min((np.abs(self.current_omega) -
                                                    accel_per_loop/self.stern_offset,0))
          speed = np.min((self.current_speed - accel_per_loop, 0))
          twist = Twist()
          twist.angular.z = omega
          twist.linear.x = speed*np.cos(angle)
          twist.linear.y = speed*np.sin(angle)
          self.publisher.publish(twist)
    # publish a final 0
    self.publisher.publish(Twist())

  def is_running(self):
    return self.running

  def stop(self):
    self.stop_requested = True

  def estop(self):
    self.deceleration = self.stop_deceleration
    self.stop_requested = True

#these execute functions should return this exception if they timeout
#timeout should indicate catastrophic failure of drive system, use accordingly
class TimeoutException(BaseException):
  pass

def simple_motion_service( motion, request ):
    velocity = None
    acceleration = None
    if request.velocity != request.DEFAULT:
        velocity = request.velocity
    if request.acceleration != request.DEFAULT:
        acceleration = request.acceleration
    try:
        if request.type == request.STRAFE:
            err = motion.execute_strafe( request.angle,
                    request.distance,
                    velocity, acceleration)
        elif request.type == request.SPIN:
            err = motion.execute_spin( request.angle, velocity, acceleration )
        response = SimpleMotionResponse( True, err )
    except TimeoutException:
        response = SimpleMotionResponse( False, 0 )
    return response

if __name__ == "__main__":
  rospy.init_node('simple_motion')
  motion = SimpleMover()
  server = rospy.Service('simple_motion',
          SimpleMotion,
          lambda req, motion=motion:simple_motion_service(motion, req ))
  rospy.spin()
