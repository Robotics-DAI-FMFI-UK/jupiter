#!/usr/bin/env python
import rospy
from matus_showcase.msg import ClothesMessage
from talk_back import *

class PersonRecognizer:
    def __init__(self):
        self.people = dict()
        self.recognized_clothing = ""
        rospy.init_node('people_recognizer')
        rospy.loginfo("Person recognition node started")
        rospy.Subscriber('clothes', ClothesMessage, self.clothes_callback)
        self.said_hello = False

        while not rospy.is_shutdown():
            rospy.sleep(1)
        rospy.loginfo("Shutting down person recognizer....")

            
    def recognize_person(self):
        if not self.known_person():
            self.add_person()
        self.say_hello()

    def clothes_callback(self, data):
        if data.glasses:
            self.recognized_clothing = "glasses"
        if data.cap:
            self.recognized_clothing = "cap"
        elif data.red_t_shirt:
            self.recognized_clothing = "red_t_shirt"
        self.said_hello = False
        self.recognize_person()

    def say_hello(self):
        
        if not self.said_hello:
            person = self.people[self.recognized_clothing]
            talker = Talker(self.recognized_clothing)
            talker.talk()
            talker = Talker("Hello " + person)
            talker.talk()
            self.said_hello = True

    def known_person(self):
        return self.recognized_clothing in self.people

    def add_person(self):
        talker = Talker("Please enter your name below:")
        talker.talk()
        name = raw_input("YOUR NAME HERE:")
        self.people[self.recognized_clothing] = name


if __name__=="__main__":
    try:
        recognizer = PersonRecognizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
