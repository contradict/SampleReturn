import smach

#the top level executive passes the level one state machine object to this
#function.  This is where its states are added
def AddStates(state_machine, announcer):
    
    with state_machine:
        
            smach.StateMachine.add('START_LEVEL_ONE',
                                   StartLevelOne(announcer),
                                   transitions = {'next':'DRIVE_TO_SAMPLE'})
            
            smach.StateMachine.add('DRIVE_TO_SAMPLE',
                                   DriveToSample(),
                                   transitions = {'sample_detected':'PURSUE_SAMPLE',
                                                  'preempted':'LEVEL_ONE_PREEMPTED'})
            
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
class StartLevelOne(smach.State):
    def __init__(self, announcer):
        smach.State.__init__(self,
                             outcomes=['next'])
        
        self.announcer = announcer
        
    def execute(self, userdata):
        
        self.announcer.say("Entering level one mode")

class DriveToSample(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['sample_detected', 'preempted']
                            )
    def execute(self, userdata):
        
        while True:
            rospy.sleep(0.1)
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
        
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
        
        #shutdown all internal processes to level_one
        
        return 'complete'

class LevelOneAborted(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['recover','fail'])
        
    def execute(self):
        
        return 'fail'