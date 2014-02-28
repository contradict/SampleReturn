import smach

#the top level executive passes the level one state machine object to this
#function.  This is where its states are added
def AddStates(state_machine):
    
    with state_machine:
        
            smach.StateMachine.add('JOYSTICK_DRIVING',
                                   JoystickDriving(),
                                   transitions = {'visual_servo_requested':'VISUAL_SERVO',
                                                  'manipulator_grab_requested':'MANIPULATOR_GRAB',
                                                  'preempted':'MANUAL_PREEMPTED',
                                                  'aborted':'MANUAL_ABORTED'})           
           
            
            smach.StateMachine.add('VISUAL_SERVO',
                                   VisualServo(),
                                   transitions = {'complete':'JOYSTICK_DRIVING',
                                                  'preempted':'MANUAL_PREEMPTED',
                                                  'aborted':'MANUAL_ABORTED'})
            
            smach.StateMachine.add('MANIPULATOR_GRAB',
                                   ManipulatorGrab(),
                                   transitions = {'complete':'JOYSTICK_DRIVING',
                                                  'preempted':'MANUAL_PREEMPTED',
                                                  'aborted':'MANUAL_ABORTED'})
            
            smach.StateMachine.add('MANUAL_PREEMPTED',
                                    ManualPreempted(),
                                    transitions = {'complete':'preempted',
                                                  'fail':'aborted'})
            
            smach.StateMachine.add('MANUAL_ABORTED',
                                   ManualAborted(),
                                   transitions = {'recover':'JOYSTICK_DRIVING',
                                                  'fail':'aborted'})
    
#drives to the expected sample location    
class JoystickDriving(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['visual_servo_requested',
                                       'manipulator_grab_requested',
                                       'preempted',
                                       'aborted'])
        
    def execute(self, userdata):
        
        return 'sample_detected'

#drive to detected sample location        
class VisualServo(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['complete', 'preempted', 'aborted'])
        
    def execute(self, userdata):
    
        return 'complete'
    
class ManipulatorGrab(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['complete', 'preempted', 'aborted'])
        
    def execute(self, userdata):
        
        return 'complete'
    
class ManualPreempted(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['complete','fail'])
        
    def execute(self):
        
        return 'complete'

class ManualAborted(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['recover','fail'])
        
    def execute(self):
        
        return 'fail'
    
    