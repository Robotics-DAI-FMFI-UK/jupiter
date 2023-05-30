#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64
from matus_showcase.msg import ObjectCoordinates

class CoordinatesCalculator:

    def __init__(self):

        rospy.init_node('arm_coordinates_publisher', anonymous=False)
        rospy.loginfo("Arm coordinates publisher was initialized.")
        rospy.on_shutdown(self.shutdown)

        # load image
        img = cv2.imread('/home/mustar/jupiter/matus/matus_showcase/images/photo.png')

        # image dimensions
        self.image_width, self.image_height, self.channels = img.shape
        print("Image dimensions: ",self.image_width, self.image_height)
        self.min_waist_angle = -0.75
        self.max_waist_angle = 0.75

        self.bbox = (0,0,0,0)
        self.center_x = 0
        self.center_y = 0
        rospy.Subscriber('objectCoordinates', ObjectCoordinates, self.coordinates_callback)
        rospy.sleep(1)

        self.waist_angle_pub = rospy.Publisher('waistAngle', Float64, queue_size=10)

        while not rospy.is_shutdown():
            self.publish_waist_angle()


    def publish_waist_angle(self):
        waist_angle = self.calculate_waist_turn()
        
        msg = Float64()
        msg.data = waist_angle

        self.waist_angle_pub.publish()

    def calculate_waist_turn(self):

        # Define the range of angles for the arm
        min_waist_angle = self.min_waist_angle
        max_waist_angle = self.max_waist_angle
        # Define the range of x-coordinates for the picture
        min_img_x = 0
        max_img_x = self.image_width - 1 

        # Compute the slope and y-intercept of the line that maps x-coordinates to arm angles
        slope = (max_waist_angle - min_waist_angle) / (max_img_x - min_img_x)
        y_intercept = min_waist_angle - slope * min_img_x

        # Given an x-coordinate, compute the corresponding angle of the arm
        # return the value of waist angle turn
        return slope * self.center_x + y_intercept

    def coordinates_callback(self, coordinates_msg):
        
        x_min = coordinates_msg.x1
        y_min = coordinates_msg.y1
        x_max = coordinates_msg.x2
        y_max = coordinates_msg.y2

        self.bbox = (x_min, y_min, x_max, y_max)

        self.center_x = (x_min + x_max) / 2
        self.center_y = (y_min + y_max) / 2

    def shutdown(self):
        rospy.loginfo('Stop arm coordinates publisher.')

if __name__ == '__main__':
    calculator = CoordinatesCalculator()
    rospy.spin()