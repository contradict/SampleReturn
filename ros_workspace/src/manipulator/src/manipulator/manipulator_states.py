# this is where all the states for the manipulator will be defined.

import smach
from manipulator.msg import ManipulatorGrabFeedback


class ProcessGoal(smach.State):
  def __init__(self):
    smach.State.__init__(self,
        outcomes=['succeeded', 'invalid_goal'],
        input_keys=['goal'],
        output_keys=['wrist_angle', 'target_bin', 'grip_torque', 'grip_dimension', 'action_feedback']
    )
  def execute(self, userdata):
    # set the feedback key
    fb = ManipulatorGrabFeedback()
    fb.current_state = "Processing Goal"
    userdata.action_feedback = fb

    # ensure that the goal is valid 
    if userdata.goal.wrist_angle > 1.7: 
      return 'invalid_goal'

    # set the outputs
    userdata.wrist_angle = userdata.goal.wrist_angle
    userdata.target_bin = userdata.goal.target_bin
    userdata.grip_torque = userdata.goal.grip_torque
    userdata.grip_dimension = userdata.goal.grip_dimension
    

    return 'succeeded' 

class ErrorState(smach.State):
  # handle any errors that show up
  def __init__(self):
    smach.State.__init__(self, outcomes=['failure', 'succeeded'] )
    
  def execute(self, userdata):
    # tell the dataStore we had an error, if it didn't already know
    if not self.userdata.error:
      return 'succeeded'
    # well did we succeed or fail at failing?
    return 'failure'

