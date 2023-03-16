#!/usr/bin/env python
import rospy
# import detect_obstacles
from geometry_msgs.msg import Twist

class MoveAround():

    def __init__(self):
        rospy.init_node('forwardPublisher', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("To stop Jupyter press CTRL + C")
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.move = Twist()
        self.freePathsListener = rospy.Subscriber('freePaths', DetectedObstacles, self.freePathCallback)
        rate = rospy.Rate(2)

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
            if obstacleInFront:
                self.moveForward()
            else:
                self.choosePath()
            # self.turnLeft()
            rospy.sleep(0)
    
    def obstacleInFront(self):
        
        return not self.directionObstacles['Front']
    
    def shutdown(self):

        rospy.loginfo("Stop Jupyter")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    def moveForward(self):

        self.move.linear.x = 0.2
        self.move.linear.z = 0
        self.cmd_vel.publish(self.move)

    def moveBackward(self):

        self.move.linear.x = -0.2
        self.move.linear.z = 0
        self.cmd_vel.publish(self.move)

    def turnLeft(self):

        self.move.linear.x = 0.8
        self.move.angular.z = 1
        self.cmd_vel.publish(self.move)

    def turnRight(self):

        self.move.linear.x = 0.8
        self.move.angular.z = -1
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
        elif self.directionObstacles['FrontRight']:
            self.turnLeft()
        elif self.directionObstacles['Left']:
            self.turnRight()
        elif self.directionObstacles['Right']:
            self.turnLeft()

if __name__ == '__main__':
    MoveAround()
    # try:
    #     MoveAround()
    # except:
    #     rospy.loginfo("MoveForward node terminated.")