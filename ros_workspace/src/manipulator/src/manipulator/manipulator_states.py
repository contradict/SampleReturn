# this is where all the states for the manipulator will be defined.

import smach
from manipulator.msg import ManipulatorFeedback, ManipulatorResult


class ProcessGoal(smach.State):
  def __init__(self):
    smach.State.__init__(self,
                        outcomes=['grab', 'home', 'preempted', 'aborted'],
                        input_keys=['action_goal'],
                        output_keys=['wrist_angle',
                                     'target_bin',
                                     'grip_torque',
                                     'grip_dimension',
                                     'action_feedback',
                                     'action_result',
                                     'error']
    )
  def execute(self, userdata):
    # set the feedback key
    fb = ManipulatorFeedback()
    fb.current_state = "START"
    userdata.action_feedback = fb

    mgr = ManipulatorResult()      
    userdata.action_result = mgr #this seems kinda lame, but set result to 
                                 #the good thing, if an error occurs it is overwritten     
    
    if userdata.action_goal.type == 'home':
      mgr.result = 'homed'
      mgr.homed = True
      return 'home'
    else:
      mgr.result = 'grab complete'
     
    # ensure that the goal is valid 
    userdata.error = None
    if userdata.action_goal.wrist_angle > 1.7: 
      userdata.error = 'invalid_goal'
      return 'aborted'
    
    # set the outputs
    userdata.wrist_angle = userdata.action_goal.wrist_angle
    userdata.target_bin = userdata.action_goal.target_bin
    userdata.grip_torque = userdata.action_goal.grip_torque
    userdata.grip_dimension = userdata.action_goal.grip_dimension
    
    return 'grab' 

class ErrorState(smach.State):
  # handle any errors that show up
  def __init__(self):
    smach.State.__init__(self,
                         input_keys=['action_result', 'error'],
                         output_keys=['action_result'],
                         outcomes=['error'] )
        
  def execute(self, userdata):
    error = 'service or action error'
    if userdata.error is not None:
      error = userdata.error
    userdata.action_result.result = error
    return 'error'
  
class PausedState(smach.State):
  
  def __init__(self, pauseCV):
    smach.State.__init__(self,
                         input_keys = ['action_goal','action_result', 'action_feedback'],
                         output_keys = ['action_result', 'action_feedback'],
                         outcomes=['succeeded', 'preempted', 'aborted'])
    self.pauseCV = pauseCV
 
  def execute(self, userdata):
    
    userdata.action_result.result = 'paused'
    userdata.action_feedback.current_state = 'PAUSED'
    
    if (userdata.action_goal.type == 'home'):
      return 'preempted'    
    else:
      self.pauseCV.acquire()
      self.pauseCV.wait()
      self.pauseCV.release()   
      return 'succeeded'
 