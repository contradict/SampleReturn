import rospy
import threading
import array

import time

from dynamixel_driver.dynamixel_const import *
from dynamixel_controllers.joint_controller_mx import JointControllerMX

from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState

from platform_motion_msgs.srv import Enable, EnableRequest

from manipulator_msgs.srv import VelocityStandoff
from manipulator_msgs.srv import TorqueStandoff
from manipulator_msgs.srv import TorqueHold
from manipulator_msgs.srv import GoToPosition

# Decorator for services that should exit when this
# controller has received a pause request.
def check_pause(fn):
    def check(*args):
        if args[0].paused: return 'preempted' #do not execute service if paused
        for result in fn(*args):
            if result is not None: #block() returns preempted or None
                return result
        return "succeeded"    
    return check

class ManipulatorJointController(JointControllerMX):
    def __init__(self, dxl_io, controller_namespace, port_namespace):
        JointControllerMX.__init__(self, dxl_io, controller_namespace, port_namespace)
      
        self.name = controller_namespace
        
        self.velocity_standoff_service = rospy.Service(self.controller_namespace + '/velocity_standoff', VelocityStandoff, self.velocity_standoff)
        self.torque_standoff_service = rospy.Service(self.controller_namespace + '/torque_standoff', TorqueStandoff, self.torque_standoff)
        self.torque_hold_service = rospy.Service(self.controller_namespace + '/torque_hold', TorqueHold, self.torque_hold)
        self.go_to_position_service = rospy.Service(self.controller_namespace + '/go_to_position', GoToPosition, self.service_go_to_position)
        self.pause_service = rospy.Service(self.controller_namespace + '/pause', Enable, self.service_pause)
                        
        #condition variable thingy to control waiting
        self.waitCV = threading.Condition()
        
        #minimum moving velocity for an "is moving" condition check
        #XXX TODO: GET THIS FROM ROS PARAMETERS
        self.min_moving_velocity = None
        self.velocity_start_delay = 0.5
        self.move_timeout = 2.0
        
        self.max_stopped_velocity = .01
        self.position_tol = .05
        
        #create an array to smooth important state variables
        #to ensure no false state detections
        self.velocity_array = array.array('f', (0,)*3)
        self.current_array = array.array('f', (0,)*3)
        self.avg_current = 0.0
        self.avg_velocity = 0
        self.goal_position = 0.0
        
        self.check_for_move = False
        self.check_for_stop = False
        self.check_for_position = False
        self.check_for_timeout = False
        self.check_current = None
        self.timeout = None
        self.timed_out = False
        self.paused = True #start up these dynamixels paused
    
        #record initial torque limit, to be reset after pauses
        self.initial_torque_limit = self.torque_limit
    
    @check_pause            
    def velocity_standoff(self, req):
        rospy.logdebug("DYNAMIXEL velocity standoff requested: {!s}".format(req))        
        velocity = req.velocity
        torque_limit = req.torque_limit
        start_torque_limit = req.start_torque_limit
        if velocity > 0:
            standoff = req.distance * -1
        else:
            standoff = req.distance
        self.set_torque_limit(start_torque_limit)
        self.set_angle_limits(0, 0) #enable wheel mode!
        self.set_speed(velocity)
        #allow a moment for dynamixel to start
        rospy.sleep(self.velocity_start_delay)
        #check to make sure the joint starts moving        
        if (req.check_velocity):
            self.min_moving_velocity = abs(velocity/2.0)        
            rospy.logdebug("DYNAMIXEL velocity standoff checking for velocity: {:f}".format(self.min_moving_velocity))
            self.check_for_move = True        
            self.start_timeout_check(self.move_timeout)
            yield self.block()
            self.check_for_timeout = False
        self.set_torque_limit(torque_limit)        
        #wait for stop        
        rospy.logdebug("DYNAMIXEL velocity standoff waiting for stop")        
        self.check_for_stop = True
        yield self.block()
        standoff_pos = self.joint_state.current_pos + standoff
        self.set_speed(0)
        self.set_angle_limits(self.min_cw_limit, self.max_ccw_limit) #back to position mode
        self.set_torque_limit(self.torque_limit)
        self.set_speed(self.joint_speed)
        self.set_position(standoff_pos)
        self.check_for_position = True
        yield self.block()
        
    
    @check_pause           
    def torque_standoff(self, req):
        torque = req.torque
        if torque > 0:
            standoff = req.distance * -1
        else:
            standoff = req.distance
        self.set_torque_control_mode_enable(True)
        self.set_goal_torque(torque)
        time.sleep(.5) #allow .5 seconds for dynamixel to move
        self.check_for_stop = True
        yield self.block()
        self.set_torque_control_mode_enable(False)
        yield self.go_to_position(self.joint_state.current_pos + standoff)
    
    @check_pause        
    def torque_hold(self, req):
        torque = req.torque
        self.set_torque_control_mode_enable(True)
        self.set_goal_torque(torque)
        self.check_current = torque * 0.9 
        yield self.block()
        self.check_for_stop = True
        yield self.block()
        yield ("succeeded")
    
    @check_pause    
    def go_to_position(self, position):
        if self.torque_control_mode: self.set_torque_control_mode_enable(False)
        self.set_position(position)            
        self.check_for_position = True
        yield self.block()
                
    def service_go_to_position(self, req):
        return self.go_to_position(req.position)

    def start_timeout_check(self, duration):
        self.timed_out = False        
        self.timeout_start_time = rospy.Time.now()
        self.timeout = rospy.Duration(duration)
        self.check_for_timeout = True

    def service_pause(self, req):
        self.paused = req.state
        if self.paused:
            self.unblock()
            self.clear_check_flags()
            self.set_torque_enable(False)
            self.set_angle_limits(self.min_cw_limit, self.max_ccw_limit) # position mode
            self.set_torque_limit(self.initial_torque_limit)
            self.set_speed(self.joint_speed)
            self.set_torque_enable(True)
            self.go_to_position(self.joint_state.current_pos)
        return self.paused
    
    def clear_check_flags(self):
        self.check_for_move = False
        self.check_for_stop = False
        self.check_for_position = False
        self.check_for_timeout = False
        self.check_current = None
          
    def process_motor_states(self, state_list):
        JointControllerMX.process_motor_states(self, state_list)
        
        # this block of crap creates the running average of velocity
        self.velocity_array.pop(0)
        self.velocity_array.append(self.joint_state.velocity)
        self.avg_velocity = sum(self.velocity_array) / len(self.velocity_array)

        # this block of crap creates the running average of current
        self.current_array.pop(0)
        self.current_array.append(self.joint_state.current)
        self.avg_current = sum(self.current_array)/len(self.current_array)         
                
        # if we have any waiting flags set, see if the wait should be cleared
        if self.check_for_move:
            if abs(self.avg_velocity) > self.min_moving_velocity:
                self.unblock()
                self.check_for_move = False
                 
        if self.check_for_stop:
            if abs(self.avg_velocity) < self.max_stopped_velocity:
                self.unblock()
                self.check_for_stop = False
            
        if self.check_for_position:
            if abs(self.joint_state.error) < self.position_tol:
                self.unblock()
                self.check_for_position = False
        
        if self.check_for_timeout:
            if (rospy.Time.now() - self.timeout_start_time) > self.timeout:
                self.timed_out = True                
                self.unblock()
                self.check_for_timeout = False

        if self.check_current is not None:
            if abs(self.avg_current) > self.check_current:
                self.unblock()
                self.check_current = None
        
                
    def block(self):
        self.waitCV.acquire()
        self.waitCV.wait()
        if self.paused:
            result = 'preempted'
        elif self.timed_out:
            result = 'timeout'
        else:
            result = None
        self.waitCV.release()
        return result

    def unblock(self):
        self.waitCV.acquire()
        self.waitCV.notifyAll()
        self.waitCV.release()
        
    
        
    
            
         