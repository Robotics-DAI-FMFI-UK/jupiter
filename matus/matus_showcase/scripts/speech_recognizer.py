#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import speech_recognition as sr

def SpeechRecognizer():
    rospy.init_node('SpeechRecognizer', anonymous=True)
    pub = rospy.Publisher('voiceCommand', String, queue_size=10)

    # Initialize the recognizer
    r = sr.Recognizer()
    
    rospy.loginfo('Google SR node is now listening...')

    # obtain audio from the microphone
    with sr.Microphone() as source:
        audio = r.listen(source, timeout=5)

    # recognize speech using Google Speech Recognition
    try:
        result = r.recognize_google(audio)
        rospy.loginfo("SR result: %s", result)
        pub.publish(result)
    except sr.UnknownValueError:
        pub.publish("!Error")
        rospy.loginfo("SR could not understand audio")
    except sr.RequestError as e:
        pub.publish("!Error")
        rospy.logerr("Could not request results from Google Speech Recognition service; %s", e)


if __name__ == '__main__':
    try:
        SpeechRecognizer()
    except rospy.ROSInterruptException:
        pass
