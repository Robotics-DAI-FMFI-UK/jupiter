#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from matus_showcase.msg import DetectedObstacles

class MoveForward():
    def __init__(self):
        rospy.init_node('forward_publisher', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("To stop Jupyter press CTRL + C")
        self.movement_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.move = Twist()
        self.freePathsListener = rospy.Subscriber('freePaths', DetectedObstacles, self.free_path_callback)

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
            self.choose_path()

    # return true if obstacle is in direction
    # return false if no obstacle is in direction
    # directions: "Front", "FrontRight", "Right", "BackRight", "Back", "BackLeft", "Left", "FrontLeft"
    def obstacle_in_direction(self, direction):
        return self.directionObstacles[direction]
    
    def shutdown(self):
        rospy.loginfo("Stop Jupyter")
        self.movement_publisher.publish(Twist())
        rospy.sleep(1)

    def stop_moving(self): 
        self.move.linear.x = 0
        self.move.angular.z = 0
        self.movement_publisher.publish(self.move)

    def move_forward(self):
        self.move.linear.x = 0.2
        self.move.angular.z = 0
        self.movement_publisher.publish(self.move)

    def move_backwards(self):
        self.move.linear.x = -0.2
        self.move.angular.z = 0
        self.movement_publisher.publish(self.move)

    def turn_left(self):
        self.move.linear.x = 0
        self.move.angular.z = 0.5
        self.movement_publisher.publish(self.move)

    def turn_right(self):
        self.move.linear.x = 0
        self.move.angular.z = -0.5
        self.movement_publisher.publish(self.move)

    def free_path_callback(self, data):
        self.directionObstacles['Front'] = bool(data.front)
        self.directionObstacles['FrontRight'] = bool(data.frontRight)
        self.directionObstacles['Right'] = bool(data.right)
        self.directionObstacles['BackRight'] = bool(data.backRight)
        self.directionObstacles['Back'] = bool(data.back)
        self.directionObstacles['BackLeft'] = bool(data.backLeft)
        self.directionObstacles['Left'] = bool(data.left)
        self.directionObstacles['FrontLeft'] = bool(data.frontLeft)

    def choose_path(self):
        if not self.obstacle_in_direction("Front"):
            self.move_forward()
        elif self.obstacle_in_direction('FrontLeft') and not self.obstacle_in_direction('FrontRight'):
            self.turn_right()
        elif self.obstacle_in_direction('FrontRight') and not self.obstacle_in_direction('FrontLeft'):
            self.turn_left()
        elif self.obstacle_in_direction('Left') and not self.obstacle_in_direction('Right'):
            self.turn_right()
        elif self.obstacle_in_direction('Right') and not self.obstacle_in_direction('Left'):
            self.turn_left()

if __name__ == '__main__':
    try:
        MoveForward()
    except:
        rospy.loginfo("MoveForward node terminated.")