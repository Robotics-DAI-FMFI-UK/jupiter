#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from matus_showcase.msg import DetectedObstacles
from math import *


class DetectObstacles:
    def __init__(self):
        rospy.init_node('obstacle_publisher', anonymous=False)
        rospy.loginfo("To stop Jupyter press CTRL + C")
        rospy.on_shutdown(self.shutdown)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # initilaze publisher with custom message
        self.pub = rospy.Publisher('freePaths', DetectedObstacles, queue_size=10)

        self.detectionRange = 0.5
        self.robotPartsRange = 0.16

        # directionRanges = [front, frontRight, right, backRight, back, backLeft, left, frontLeft]
        self.directionRanges = [[] for _ in range(8)]
        self.directionObstacles = {
            "Front" : False,
            "FrontRight" : False,
            "Right" : False,
            "BackRight" : False,
            "Back" : False,
            "BackLeft" : False,
            "Left" : False,
            "FrontLeft" : False
        }
        while not rospy.is_shutdown():
            self.publish_paths_message()

    def shutdown(self):
        rospy.loginfo('Stop jupyter')

    def scan_callback(self, msg):
        self.directionRanges[0] = msg.ranges[339:359] + msg.ranges[0:19]
        angle = 45
        for i in range(len(self.directionRanges) - 1):
            self.directionRanges[i+1] = msg.ranges[angle - 20 : angle + 20]
            angle += 45
        self.check_distances()

    def check_distances(self):
        directions = ['Front', 'FrontRight', 'Right', 'BackRight',
                    'Back', 'BackLeft', 'Left', 'FrontLeft']

        for index, direction in enumerate(directions):
            obstacle = False
            warning = False
            for i in self.directionRanges[index]:
                if not isinf(i):
                    if self.robotPartsRange < i <= self.detectionRange:
                        warning = True
                        if warning:
                            obstacle = True
                            break
            self.directionObstacles[direction] = obstacle

    def possible_paths(self):
        return [i[0] for i in self.directionObstacles.items() if not i[1]]
    
    #publishujem svoj custom message
    def publish_paths_message(self):
        msg = DetectedObstacles()
        msg.front = int(self.directionObstacles["Front"])
        msg.frontRight = int(self.directionObstacles["FrontRight"])
        msg.right = int(self.directionObstacles["Right"])
        msg.backRight = int(self.directionObstacles["BackRight"])
        msg.back = int(self.directionObstacles["Back"])
        msg.backLeft = int(self.directionObstacles["BackLeft"])
        msg.left = int(self.directionObstacles["Left"])
        msg.frontLeft = int(self.directionObstacles["FrontLeft"])
        self.pub.publish(msg)

if __name__ == '__main__':
    try:
        DetectObstacles()
    except:
        rospy.loginfo("obstaclePublisher node terminated.")