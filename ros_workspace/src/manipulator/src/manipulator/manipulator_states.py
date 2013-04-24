# this is where all the states for the manipulator will be defined.

import smach
from manipulator.msg import ManipulatorGrabFeedback


class ProcessGoal(smach.State):
  def __init__(self):
    smach.State.__init__(self,
                        outcomes=['succeeded', 'aborted'],
                        input_keys=['goal'],
                        output_keys=['wrist_angle', 'target_bin', 'grip_torque', 'grip_dimension', 'action_feedback', 'result']
    )
  def execute(self, userdata):
    # set the feedback key
    fb = ManipulatorGrabFeedback()
    fb.current_state = "Processing Goal"
    userdata.action_feedback = fb

    # ensure that the goal is valid 
    if userdata.goal.wrist_angle > 1.7: 
      userdata.result = 'invalid_goal'
      return 'aborted'
    else:
      userdata.result = 'cycle complete' #this seems kinda lame, but set result to 
                                         #the good thing, if an error occurs it is overwritten     

    # set the outputs
    userdata.wrist_angle = userdata.goal.wrist_angle
    userdata.target_bin = userdata.goal.target_bin
    userdata.grip_torque = userdata.goal.grip_torque
    userdata.grip_dimension = userdata.goal.grip_dimension
    
    return 'succeeded' 

class ErrorState(smach.State):
  # handle any errors that show up
  def __init__(self):
    smach.State.__init__(self,
                         input_keys=['result'],
                         output_keys=['result'],
                         outcomes=['error'] )
        
  def execute(self, userdata):
    # if result has been filled prior to reaching error
    # it should be the reason for the error
    if userdata.result is None:
      userdata.result = 'service or action error'
      # if not, it failed on a service or action
    return 'error'
  
class PausedState(smach.State):
  
  def __init__(self, pauseCV):
    smach.State.__init__(self,
                         input_keys = ['result'],
                         output_keys = ['result'],
                         outcomes=['succeeded', 'aborted'])
    self.pauseCV = pauseCV
 
  def execute(self, userdata):
    
    userdata.result = 'paused'
    
    self.pauseCV.acquire()
    self.pauseCV.wait()
    self.pauseCV.release()   
    
    return 'succeeded'
  
  #code


