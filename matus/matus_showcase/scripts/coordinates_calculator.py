#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64
from matus_showcase.msg import ObjectCoordinates

'''
    real world dimensions:
    width = 40 cm
    height = 30 cm
    depth = 32 cm
    

    image
    width = 640 pixel
    height = 480 pixel

    camera fov:
        alpha = 49.5
        beta = 60

    arm_length = 10.5
    forearm_length = 10.5
    hand_length = 12

    bbox of the cup (393, 179, 485, 276)
'''

class CoordinatesCalculator:

    def __init__(self):

        rospy.init_node('armCoordinatesPublisher', anonymous=False)
        rospy.loginfo("Arm coordinates publisher was initialized.")
        rospy.on_shutdown(self.shutdown)

        # manipulator dimensions
        self.arm_length = 10.5
        self.forearm_length = 10.5
        self.hand_length = 12

        
        # real world dimensions in centimeters
        self.width_real = 40
        self.height_real = 30
        self.depth_real = 32

        # load image
        img = cv2.imread('/home/mustar/jupiter/matus/matus_showcase/images/photo.png')

        # image dimensions
        self.width_img, self.height_img, self.channels = img.shape
        print(self.width_img, self.height_img)
        self.object_center_x = -1
        self.object_center_y = -1

        # get the center coordinates from topic  ObjectCoordinates
        # sleep for 1 second to give enough time for calculation
        rospy.Subscriber('objectCoordinates', ObjectCoordinates, self.coordinates_callback)
        rospy.sleep(1)
        print(self.object_center_x, self.object_center_y)

        # camera FOV alpha is vertical, beta is horizontal
        self.alpha = np.deg2rad(49.5)
        self.beta = np.deg2rad(60)

        self.real_y = self.calculate_real_y()
        self.real_x = self.calculate_real_x()
        self.cosine_theorem_result()

    def calculate_real_y(self):
        y = (self.object_center_y / self.height_img) * 2 * self.depth_real * np.tan(self.alpha / 2)
        print(y)
        return y

    def calculate_real_x(self):
        x = (self.object_center_x / self.width_img) * 2 * self.depth_real * np.tan(self.beta / 2)
        print(x)
        return x

    def cosine_theorem_result(self):
        xx = self.real_x - self.hand_length * np.cos(np.deg2rad(45))
        yy = self.real_y - self.hand_length * np.sin(np.deg2rad(45))
        AC = np.sqrt(xx**2 + yy**2)
        print("HEHEHE> ",AC)
        
        arm_degree = np.pi/2 - np.arctan(yy/xx) - np.arccos((self.arm_length**2 + AC**2 - self.forearm_length**2) / 2*self.arm_length*AC)
        print(arm_degree)

    def coordinates_callback(self, coordinates_msg):
        
        x1 = coordinates_msg.x1
        y1 = coordinates_msg.y1
        x2 = coordinates_msg.x2
        y2 = coordinates_msg.y2

        # center coordinates of the object in image
        x = (x2 - x1) / 2
        y = (y2 - y1) / 2

        self.object_center_x = x
        self.object_center_y = y


    def shutdown(self):
        rospy.loginfo('Stop arm coordinates publisher.')

if __name__ == '__main__':
    calculator = CoordinatesCalculator()
    rospy.spin()