#!/usr/bin/env python

import rospy
from matus_showcase.msg import ClothesMessage

class ClothingPublisher:

    # Initialize the node
    def __init__(self, clothing):
        rospy.init_node('clothing_publisher', anonymous=False)
        rospy.loginfo("Clothing publisher was initialized.")
        rospy.on_shutdown(self.shutdown)
        self.clothing = clothing

        self.pub = rospy.Publisher('clothes', ClothesMessage, queue_size=10)
        
        self.publish_clothes_message()
        rospy.sleep(1)
        rospy.signal_shutdown("STOP")
    
    def publish_clothes_message(self):
        msg = ClothesMessage
        msg.glasses = True if self.clothing == "glasses" else False
        msg.cap = True if self.clothing == "cap" else False
        msg.red_t_shirt = True if self.clothing == "red_t_shirt" else False
        self.pub.publish(msg)

    def shutdown(self):
        rospy.loginfo('Stop clothing publisher')
