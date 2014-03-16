#!/usr/bin/python
import rospy
import sys
import threading
sys.path.append('/opt/ros/groovy/stacks/audio_common/sound_play/src/')
from sound_play.msg import SoundRequest

from samplereturn_msgs.msg import VoiceAnnouncement


class Announcer(object):
    
    def __init__(self):
        rospy.init_node("announcer", anonymous=True)
        
        self.voice = rospy.get_param("~voice", "kal_diphone")
        self.speech_delay = rospy.get_param("~speech_delay", 0.8)
        self.speech_rate = rospy.get_param("~speech_rate", 0.07)        
    
        self.audio_sub=rospy.Subscriber("audio_in",
                                        platform_msg.VoiceAnnouncement,
                                        self.handle_announcement)
        
        self.audio_pub=rospy.Publisher("audio_out", SoundRequest)
        
        self.announcements = []
        self.announceCV = threading.Condition()
                
        self.announcement_worker()
                
        rospy.spin()
    
    def handle_announcement(self, announcement):

        self.announceCV.acquire()        
        if announcement.priority == announcement.LAST:
            self.announcements.append(announcement.words)
        elif announcement.priority == announcement.FIRST:
            self.announcements.insert(0, announcement.words)
        elif announcement.priority == announcement.OVERRIDE:
            self.announcements = [announcement.words]
            self.announceCV.notifyAll()
        self.announceCV.release()

    
    def announcement_worker(self):
        while True:
            self.announceCV.acquire()
            if len(self.announcements) > 0:
                msg = SoundRequest()
                msg.sound = SoundRequest.SAY
                msg.command = SoundRequest.PLAY_ONCE
                msg.arg = self.announcements[0].words
                msg.arg2 = 'voice.select "%s"'%self.voice
                self.audio_pub(msg)
                self.announcements.pop[0]
                duration = self.speech_delay + self.speech_rate*len(msg.arg)
                self.announceCV.wait(timeout=duration)
            else:
                self.announceCV.wait()
                self.announceCV.release()
        
    
if __name__=="__main__":
    Announcer()
