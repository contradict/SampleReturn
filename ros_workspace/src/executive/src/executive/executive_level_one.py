import smach

#the top level executive passes the level one state machine object to this
#function.  This is where its states are added
def AddStates(state_machine):
    
    with state_machine:
        
            smach.StateMachine.add('DRIVE_TO_SAMPLE',
                                   DriveToSample(),
                                   transitions = {'sample_detected':'PURSUE_SAMPLE'})
            
            smach.StateMachine.add('PURSUE_SAMPLE',
                                   PursueSample(),
                                   transitions = {'sample_acquired':'complete'})
            
            smach.StateMachine.add('LEVEL_ONE_PREEMPTED',
                                  LevelOnePreempted(),
                                   transitions = {'complete':'preempted',
                                                  'fail':'aborted'})
            
            smach.StateMachine.add('LEVEL_ONE_ABORTED',
                                   LevelOneAborted(),
                                   transitions = {'recover':'DRIVE_TO_SAMPLE',
                                                  'fail':'aborted'})  
    
#drives to the expected sample location    
class DriveToSample(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['sample_detected']
                            )
    def execute(self, userdata):
        
        return 'sample_detected'

#drive to detected sample location        
class PursueSample(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['sample_acquired']
                            )
        
    def execute(self, userdata):
    
        return 'sample_acquired'
    
class LevelOnePreempted(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['complete','fail'])
        
    def execute(self):
        
        return 'complete'

class LevelOneAborted(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['recover','fail'])
        
    def execute(self):
        
        return 'fail'