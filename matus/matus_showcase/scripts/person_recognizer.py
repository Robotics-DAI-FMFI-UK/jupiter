#!/usr/bin/env python
import rospy
from matus_showcase import ClothesMessage
from talk_back import *

class PersonRecognizer:
    def __init__(self):
        self.people = {}
        self.recognized_clothing = ""
        rospy.init_node('people_recognizer')
        rospy.Subscriber('clothes', ClothesMessage, self.clothes_callback)
        
        while not rospy.is_shutdown():
            rospy.sleep(1)
            
    def recognize_person(self):
        if not self.known_person():
            self.add_person()
        self.say_hello()

    def clothes_callback(self, data):
        if data.glasses:
            self.recognized_clothing = "glasses"
        if data.cap:
            self.recognized_clothing = "cap"
        else:
            self.recognized_clothing = "red_t_shirt"

        self.recognize_person()

    def say_hello(self):
        person = self.people[self.recognized_clothing]
        talker = Talker(self.recognized_clothing)
        talker.talk()
        talker = Talker("Hello ", person)
        talker.talk()

    def known_person(self):
        return self.recognized_clothing in self.people

    def add_person(self):
        talker = Talker("Please enter your name below:")
        talker.talk()
        name = input("YOUR NAME HERE:")
        self.people[self.recognized_clothing] = name

    def shutdown(self):
        rospy.loginfo("Shutting down person recognizer....")

if __name__=="__main__":
    try:
        recognizer = PersonRecognizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
