# this is where all the states for the manipulator will be defined.

import smach

class HandleGoal(smach.State):
  # this state might not be useful.
  # get the goal and stuff it in dataStore. also, output the desired wrist
  # rotation so RotateWrist can take it as input.
  def __init__(self, dataStore):
    smach.State.__init__(self,
        outcomes=['success'],
        input_keys=['goal'],
        output_keys=['wrist_angle']
    )
    self.dataStore = dataStore

  def execute(self, userData):
    # store the current goal in dataStore for safe keeping
    self.dataStore.currentActionGoal = userdata.goal

    # set the wrist angle output
    userdata.wrist_angle = userdata.goal.wrist_angle

    # success!
    return 'success'

class RotateWrist(smach.State):
  def __init__(self, dataStore):
    smach.State.__init__(self, 
        outcomes=['success', 'failure'],
        input_keys=['wrist_angle']
    )
    # save a reference to the persistant data
    self.dataStore = dataStore

  def execute(self, userdata):
    # actually make things happen

    # acquire the condition variable for the wrist
    self.dataStore.wristCV.acquire()

    # ask the dataStore to make the wrist angle correct
    self.dataStore.SetWristAngle(userdata.wrist_angle)

    # wait until the wrist gets to the right place.
    self.dataStore.wristCV.wait()

    # check to make sure we got the right wrist angle
    if abs(self.dataStore.GetWristAngle - userdata.wrist_angle) <= 0.01:
      return 'success'
    else:
      return 'failure'

class StartMovingArm(smach.State):
  # this class goes after a couple of servic states that actually get the arm
  # moving and just waits for the arm to get to a specific position
  def __init__(self, dataStore, minArmMovement=0.5):
    smach.State.__init__(self,
        outcomes=['success', 'failure']
    )
    self.dataStore = dataStore
    # store how much movement counts as enough
    self.minArmMovement = minArmMovement

  def execute(self, userdata):
    # actually do stuff, mostly just wait for the arm to move.
    # get the lock first.
    self.dataStore.armCV.acquire()
    # request a delta of like 20 degrees of movement
    self.dataStore.NotifyOnArmDelta(self.minArmMovement)

    # wait on the condition variable
    self.dataStore.armCV.wait()

    # for now, assume it always works if we get here.
    return 'success'

def WaitForArmStop(smach.State):
  # once the arm is moving, wait for it to hit stuff.
  def __init__(self, dataStore, limit):
    smach.State.__init__(self,
        outcomes=['success', 'failure']
    )
    self.dataStore = dataStore
    # limit is the torque limit we're waiting for
    self.limit = limit

  def execute(self, userdata):
    # actually do stuff, like waiting. yay waiting!

    # get the arm lock.
    self.dataStore.armCV.acquire()

    # tell the dataStore that we'd like to know about the arm's torque
    self.dataStore.NotifyOnArmTorque(self.limit)

    # now wait on the condition variable for the torque to reach the right
    # limit
    self.dataStore.armCV.wait()

    # for now, assume success always actually happens
    return 'success'

def StartMovingHand(smach.State):
  # when the hand starts closing, wait for it to move before watching for
  # torque
  def __init__(self, dataStore, minHandMovement=0.02):
    smach.State.__init__(self,
        outcomes=['success', 'failure']
    )
    self.dataStore = dataStore
    self.minHandMovement = minHandMovement

  def execute(self, userdata):
    # get the hand lock
    self.dataStore.handCV.acquire()

    # as the dataStore to notify when the hand has moved a bit
    self.dataStore.NotifyOnHandDelta(self.minHandMovement)

    # wait for notification indicating success
    self.dataStore.handCV.wait()

    # for now, assume it always worked
    return 'success'

def WaitForHandStop(smach.State):
  # when the hand is moving, wait for it to actually hit a torque limit
  def __init__(self, dataStore, limit):
    smach.State.__init__(self,
        outcomes=['success', 'failure']
    )
    self.dataStore = dataStore
    # limit is the torque limit we're waiting for
    self.limit = limit

  def execute(self, userdata):
    # get the hand lock
    self.dataStore.handCV.acquire()

    # ask for the hand torque limit notification
    self.dataStore.NotifyOnHandTorque(self.limit)

    # wait on the lock
    self.dataStore.handCV.wait()

    # assume success at this point
    return 'success'
