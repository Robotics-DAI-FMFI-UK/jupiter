#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from matus_showcase.msg import DetectedObstacles

class MoveAround():

    def __init__(self):
        rospy.init_node('forwardPublisher', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("To stop Jupyter press CTRL + C")
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.move = Twist()
        self.freePathsListener = rospy.Subscriber('freePaths', DetectedObstacles, self.freePathCallback)

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

            if not self.obstacleInFront():
                print('free')
                self.moveForward()
            else:
                print('obstacle in sight')
                self.stopMoving()
            rospy.sleep(0)
    
    def obstacleInFront(self):
        # print("VPREDU> ", self.directionObstacles['Front'])
        return self.directionObstacles['Front']
    
    def shutdown(self):

        rospy.loginfo("Stop Jupyter")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    def stopMoving(self):
        
        self.move.linear.x = 0
        self.move.linear.z = 0
        self.cmd_vel.publish(self.move)

    def moveForward(self):

        self.move.linear.x = 0.2
        self.move.linear.z = 0
        self.cmd_vel.publish(self.move)

    def moveBackward(self):

        self.move.linear.x = -0.2
        self.move.linear.z = 0
        self.cmd_vel.publish(self.move)

    def turnLeft(self):

        self.move.linear.x = 0
        self.move.angular.z = 0.5
        self.cmd_vel.publish(self.move)

    def turnRight(self):

        self.move.linear.x = 0
        self.move.angular.z = -0.5
        self.cmd_vel.publish(self.move)

    def freePathCallback(self, data):

        self.directionObstacles['Front'] = True if data.front == 1 else False
        self.directionObstacles['FrontRight'] = True if data.frontRight == 1 else False
        self.directionObstacles['Right'] = True if data.right == 1 else False
        self.directionObstacles['BackRight'] = True if data.backRight == 1 else False
        self.directionObstacles['Back'] = True if data.back == 1 else False
        self.directionObstacles['BackLeft'] = True if data.backLeft == 1 else False
        self.directionObstacles['Left'] = True if data.left == 1 else False
        self.directionObstacles['FrontLeft'] = True if data.frontLeft == 1 else False

    def choosePath(self):

        if self.directionObstacles['FrontLeft']:
            self.turnRight()
            return
        elif self.directionObstacles['FrontRight']:
            self.turnLeft()
            return
        elif self.directionObstacles['Left']:
            self.turnRight()
            return
        elif self.directionObstacles['Right']:
            self.turnLeft()
            return

if __name__ == '__main__':
    MoveAround()
    # try:
    #     MoveAround()
    # except:
    #     rospy.loginfo("MoveForward node terminated.")