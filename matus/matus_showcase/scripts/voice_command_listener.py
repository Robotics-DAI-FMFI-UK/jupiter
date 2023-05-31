#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from talk_back import Talker
import os


class VoiceCommandListener:
    
    def __init__(self):
        rospy.init_node('voice_command_listener')
        rospy.Subscriber('voiceCommand', String, self.voice_command_callback)

        # Define the command-action dictionary
        self.command_actions = {
            '!Error': self.handle_error,
            'recognize': self.start_person_recognition,
            'find object': self.search_for_cup,
            'move forward': self.move_forward
        }

    def voice_command_callback(self, msg):
        rospy.loginfo('Received voice command: %s', msg.data)

        # Check if the command exists and execute it
        if msg.data in self.command_actions:
            action = self.command_actions[msg.data]
            action()
        else:
            talk_back_text = "I don't know this command."
            self.talk_back(talk_back_text)

    def handle_error(self):
        talk_back_text = "I could not recognize the audio."
        self.talk_back(talk_back_text)

    def start_person_recognition(self):
        talk_back_text = "Starting person recognition."
        self.talk_back(talk_back_text)
        os.system('rosrun matus_showcase take_picture.py')
        os.system('rosrun matus_showcase send_image.py')

    def search_for_cup(self):
        talk_back_text = "Searching for a cup."
        self.talk_back(talk_back_text)
        os.system('rosrun matus_showcase take_picture.py _camera_topic:=/camera/rgb/image_raw')
        os.system('rosrun matus_showcase arm_coordinates_calculator.py')
        os.system('rosrun matus_showcase find_a_cup.py')
        os.system('rosrun matus_showcase find_a_cup.py')

    def move_forward(self):
        talk_back_text = "Moving forward."
        self.talk_back(talk_back_text)
        os.system('rosrun matus_showcase move_forward.py')

    def start(self):
        rospy.spin()

    def talk_back(self, text):
        talker = Talker(text)
        talker.talk()   


if __name__ == '__main__':
    vcl = VoiceCommandListener()
    vcl.start()
