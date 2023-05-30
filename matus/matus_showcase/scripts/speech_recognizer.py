#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import speech_recognition as sr

class SpeechRecognizer:

    def __init__(self):
        rospy.init_node('speech_recognizer', anonymous=True)
        # Initialize voice command publisher
        self.voice_command_publisher = rospy.Publisher('voiceCommand', String, queue_size=10)

        # Initialize the recognizer
        self.r = sr.Recognizer()
    
    def recognize(self):

        rospy.loginfo('Google SR node is now listening...')

        # obtain audio from the microphone
        with sr.Microphone() as source:
            audio = self.r.listen(source, timeout=5)

        # recognize speech using Google Speech Recognition
        # publish result to /voiceCommand topic
        try:
            result = self.r.recognize_google(audio)
            rospy.loginfo("SR result: %s", result)
            self.voice_command_publisher.publish(result)
        except sr.UnknownValueError:
            self.voice_command_publisher.publish("!Error")
            rospy.loginfo("SR could not understand audio")
        except sr.RequestError as e:
            self.voice_command_publisher.publish("!Error")
            rospy.logerr("Could not request results from Google Speech Recognition service; %s", e)

if __name__ == '__main__':
    try:
        sr = SpeechRecognizer()
        sr.recognize()
    except rospy.ROSInterruptException:
        pass
