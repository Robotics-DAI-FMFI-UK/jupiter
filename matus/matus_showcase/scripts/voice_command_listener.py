#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from talk_back import Talker
import os



class VoiceCommandListener:
    def __init__(self):
        rospy.init_node('voice_command_listener')
        rospy.Subscriber('voiceCommand', String, self.voice_command_callback)

    def voice_command_callback(self, msg):
        rospy.loginfo('Received voice command: %s', msg.data)

        if(msg.data == '!Error'):
            talk_back_text = "I could not recognize the audio."
            talker = Talker(talk_back_text)
            talker.talk()
        elif(msg.data == 'recognize'):
            talk_back_text = "Starting person recognition."
            talker = Talker(talk_back_text)
            talker.talk()
            os.chdir('/home/mustar/jupiter/matus/matus_showcase/scripts')    
            os.system('rosrun matus_showcase take_picture.py')        
            os.system('rosrun matus_showcase send_image.py')
        else:
            talk_back_text = msg.data
            talker = Talker(talk_back_text)
            talker.talk()
            # talk_back_text = "I don't know this command."
        # rospy.sleep(10)



    def start(self):
        rospy.spin()


if __name__ == '__main__':
    vcl = VoiceCommandListener()
    vcl.start()
