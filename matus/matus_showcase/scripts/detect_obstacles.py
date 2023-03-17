#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

# nefunguje z nejakeho dovodu
from matus_showcase.msg import DetectedObstacles

from math import *


class DetectObstacles:

    def __init__(self):
        rospy.init_node('obstaclePublisher', anonymous=False)
        rospy.loginfo("To stop Jupyter press CTRL + C")
        rospy.on_shutdown(self.shutdown)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        
        # init publishera ktory dava custom message
        self.pub = rospy.Publisher('freePaths', DetectedObstacles)

        self.rate = rospy.Rate(10)
        self.detectionRange = 0.8
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
            # volam publishovanie custom messagu
            self.publishPathsMessage()
            print(self.possiblePaths())
            # print(self.directionRanges[0])
            self.rate.sleep()
    
    def shutdown(self):
        
        rospy.loginfo('Stop jupyter')
        rospy.sleep(2)

    def callback(self, msg):

        self.directionRanges[0] = msg.ranges[339:359] + msg.ranges[0:19]
        angle = 45
        for i in range(len(self.directionRanges) - 1):
            self.directionRanges[i+1] = msg.ranges[angle - 20 : angle + 20]
            angle += 45
        self.checkDistances()

    def checkDistances(self):
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

    def possiblePaths(self):

        return [i[0] for i in self.directionObstacles.items() if not i[1]]
    
    #publishujem svoj custom message
    def publishPathsMessage(self):

        msg = DetectedObstacles()
        msg.front = 1 if self.directionObstacles["Front"] else 0
        msg.frontRight = 1 if self.directionObstacles["FrontRight"] else 0
        msg.right = 1 if self.directionObstacles["Right"] else 0
        msg.backRight = 1 if self.directionObstacles["BackRight"] else 0
        msg.back = 1 if self.directionObstacles["Back"] else 0
        msg.backLeft = 1 if self.directionObstacles["BackLeft"] else 0
        msg.left = 1 if self.directionObstacles["Left"] else 0
        msg.frontLeft = 1 if self.directionObstacles["FrontLeft"] else 0
        self.pub.publish(msg)

if __name__ == '__main__':
    DetectObstacles()
    # try:
    #     DetectObstacles()
    # except:
    #     rospy.loginfo("obstaclePublisher node terminated.")