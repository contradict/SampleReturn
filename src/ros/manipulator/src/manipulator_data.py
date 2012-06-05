# a class to store data for the manipulator states.
import threading

# some of the nodes need access to subscriptions and global crud
# so encapsulate it all in one datastore
class PersistantData:
  def __init__(self):
    # store the current goal, hopefully will get set right after the action
    # starts.
    self.currentActionGoal = None

    # make condition variables for the various things the states care about.
    self.wristCV = threading.Condition()
    self.wristAngleGoal = None
    self.currentWristState = None

    self.armCV = threading.Condition()
    self.armPosGoal = None
    self.armTorqueGoal = None
    self.currentArmState = None

    self.handCV = threading.Condition()
    self.handPosGoal = None
    self.handTorqueGoal = None
    self.currentHandState = None

    # I'm going to assume that the node that owns the instance of this class
    # will do the actual setup of the publishers and subscriptions, but save
    # publisher slots here for it to fill in.
    self.wristPosPublisher = None
    self.armPosPublisher = None
    self.handPosPublisher = None

  def SetWristAngle(self, angle):
    # set the wrist angle to the given value
    self.wristAngleGoal = angle

    # tell the publisher to do it
    self.wristPosPublisher.publish(self.wristAngleGoal)

  def WristJointCallback(self, data):
    # check out the state of the goal and notify anyone who may be waiting for
    # the wrist to reach a specific state.
    if self.wristAngleGoal != None:
      if abs(data.current_pos - self.wristAngleGoal) < 0.01:
        # if we have a goal and we just met it, notify all and release the
        # condition variable
        self.wristCV.acquire()
        self.wristAngleGoal = None
        self.wristCV.notifyAll()
        self.wristCV.release()

    # save the wrist state
    self.currentWristState = data

  def GetWristAngle(self):
    # return the current wrist angle
    return self.currentWristState.current_pos

  def NotifyOnArmDelta(self, delta):
    # notifies the caller (via self.armCV when the arm has moved delta or more
    self.armPosGoal = delta
    # XXX TODO: make sure this is really how arm pos is stored
    if self.currentArmState != None:
      self.armPosAtGoalStart = self.currentArmState.current_pos
    else:
      self.armPosAtGoalStart = None

  def NotifyOnArmTorque(self, torque):
    # notifies the caller (via self.armCV) when the arm reaches torqe and stops
    # moving
    self.armTorqueGoal = torque

  def ArmJointCallback(self, data):
    # listen to the state of the arm joint and do notifications as needed.
    if self.armPosGoal != None:
      if self.armPosAtGoalStart == None:
        # it may be possible to have a goal before getting any arm pos messages
        # avoid deadlock by dealing with that case here
        self.armPosAtGoalStart = data.current_pos
      elif abs(self.armPosAtGoalStart - data.current_pos) >= self.armPosGoal:
        self.armCV.acquire()
        self.armPosGoal = None
        self.armCV.notifyAll()
        self.armCV.release()

    elif self.armTorqueGoal != None:
      if (not data.is_moving) and (abs(data.load - self.armTorqueGoal) <= 0.05):
        # if we get really close to the desired arm torque, call it good enough
        # and notify.
        self.armCV.acquire()
        self.armTorqueGoal = None
        self.armCV.notifyAll()
        self.armCV.release()

    # store the arm position
    self.currentArmState = data

  def NotifyOnHandDelta(self, delta):
    # notifies the caller via self.handCV when the hand has moved delta or more
    self.handPosGoal = delta
    if self.currentHandState != None:
      self.handPosAtGoalStart = self.currentHandState.current_pos
    else:
      self.handPosAtGoalStart = None

  def NotifyOnHandTorque(self, torque):
    # notifiy the caller via self.handCV when the torque has reached the
    # desired limit and the hand has stopped moving
    self.handTorqueGoal = torque

  def HandJointCallback(self, data):
    # listen to the state of the hand joint and notify listeners as needed
    if self.handPosGoal != None:
      if self.handPosAtGoalStart == None:
        self.handPosAtGoalStart = data.current_pos
      elif abs(self.handPosAtGoalStart - data.current_pos) >= self.handPosGoal:
        # the hand has moved enough to count as having moved!
        self.handCV.acquire()
        self.handPosGoal = None
        self.handCV.notifyAll()
        self.handCV.release()

    elif self.handTorqueGoal != None:
      if (not data.is_moving) and (abs(data.load - self.handTorqueGoal) <= 0.05):
        # acquire the lock and notify all
        self.handCV.acquire()
        self.handTorqueGoal = None
        self.handCV.notifyAll()
        self.handCV.release()

    # store the current hand state
    self.currentHandState = data
