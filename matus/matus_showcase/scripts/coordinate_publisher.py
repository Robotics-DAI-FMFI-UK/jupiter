#!/usr/bin/env python

import rospy
from matus_showcase.msg import ObjectCoordinates

class CoordinatesPublisher:

    # Initialize the node
    def __init__(self, coordinates):
        rospy.init_node('coordinates_publisher', anonymous=False)
        rospy.loginfo("Object coordinates publisher was initialized.")
        rospy.on_shutdown(self.shutdown)
        self.coordinates = coordinates

        self.pub = rospy.Publisher('objectCoordinates', ObjectCoordinates, queue_size=10)
        rospy.sleep(1)

        self.publish_coordinates_message()
            
        rospy.signal_shutdown("STOP")
    
    def publish_coordinates_message(self):
        msg = ObjectCoordinates()
        x1, y1, x2, y2 = self.coordinates
        msg.x1 = x1
        msg.y1 = y1
        msg.x2 = x2
        msg.y2 = y2
        self.pub.publish(msg)

    def shutdown(self):
        rospy.loginfo('Stop object coordinates publisher')
