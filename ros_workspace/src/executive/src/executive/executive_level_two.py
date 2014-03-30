import smach

#the top level executive passes the level one state machine object to this
#function.  This is where its states are added
def AddStates(state_machine):
    
    with state_machine:
        
            smach.StateMachine.add('GLOBAL_SEARCH',
                                   GlobalSearch(),
                                   transitions = {'sample_detected':'PURSUE_SAMPLE'})
            
            smach.StateMachine.add('PURSUE_SAMPLE',
                                   PursueSample(),
                                   transitions = {'sample_acquired':'RETURN_TO_SEARCH'})
    
            smach.StateMachine.add('RETURN_TO_SEARCH',
                                   ReturnToSearch(),
                                   transitions = { 'complete':'GLOBAL_SEARCH'})

            smach.StateMachine.add('LEVEL_TWO_PREEMPTED',
                                  LevelTwoPreempted(),
                                   transitions = {'complete':'preempted',
                                                  'fail':'aborted'})
            
            smach.StateMachine.add('LEVEL_TWO_ABORTED',
                                   LevelTwoAborted(),
                                   transitions = {'recover':'GLOBAL_SEARCH',
                                                  'fail':'aborted'})                                   
    
#searches the globe   
class GlobalSearch(smach.State):
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
  
    
class ReturnToSearch(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['complete']
                            )
    def execute(self, userdata):
        
        return 'complete'
    
class LevelTwoPreempted(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['complete','fail'])
        
    def execute(self):
        
        return 'complete'

class LevelTwoAborted(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['recover','fail'])
        
    def execute(self):
        
        return 'fail'