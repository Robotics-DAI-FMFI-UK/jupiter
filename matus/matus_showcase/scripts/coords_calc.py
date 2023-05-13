#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64
from matus_showcase.msg import ObjectCoordinates

class CoordinatesCalculator:

    def __init__(self):

        rospy.init_node('armCoordinatesPublisher', anonymous=False)
        rospy.loginfo("Arm coordinates publisher was initialized.")
        rospy.on_shutdown(self.shutdown)

        # manipulator dimensions
        self.arm_length = 10.5
        self.forearm_length = 10.5
        self.hand_length = 12
        # distance between the camera is behind the arm base in cm
        self.distance_front = 15
        # distance the camera is above the arm in cm
        self.distance_above = 20

        # load image
        img = cv2.imread('/home/mustar/jupiter/matus/matus_showcase/images/photo.png')

        # image dimensions
        self.height_img,self.width_img, self.channels = img.shape
        print(self.width_img, self.height_img)

        self.x_min_norm = -1
        self.y_min_norm = -1
        self.x_max_norm = -1
        self.y_max_norm = -1
        self.object_center_x = -1
        self.object_center_y = -1


        # get the normalized bbox coordinates from topic  ObjectCoordinates
        # sleep for 1 second to give enough time for calculation
        rospy.Subscriber('objectCoordinates', ObjectCoordinates, self.coordinates_callback)
        rospy.sleep(1)
        print(self.x_min_norm, self.y_min_norm)
        print(self.x_max_norm, self.y_max_norm)

        # camera FOV alpha is vertical, beta is horizontal
        self.alpha = 49.5
        self.beta = 60

        self.horizontal_angle = -1
        self.vertical_angle = -1
        self.bounding_box_angles()
        print(self.horizontal_angle, self.vertical_angle)
        print(self.calculate_3d_coordinates())

    def calculate_3d_coordinates(self):
        x_end = np.tan(np.deg2rad(self.horizontal_angle))
        y_end = np.tan(np.deg2rad(self.vertical_angle))
        z_end = 0

        x_joint3 = x_end - self.hand_length * np.sin(np.deg2rad(self.vertical_angle)) * np.cos(np.deg2rad(self.horizontal_angle))
        y_joint3 = y_end - self.hand_length * np.sin(np.deg2rad(self.vertical_angle)) * np.sin(np.deg2rad(self.horizontal_angle))
        z_joint3 = z_end + self.hand_length * np.cos(np.deg2rad(self.vertical_angle))

        x_joint2 = x_joint3 - self.forearm_length * np.sin(np.deg2rad(self.vertical_angle)) * np.cos(np.deg2rad(self.horizontal_angle))
        y_joint2 = y_joint3 - self.forearm_length * np.sin(np.deg2rad(self.vertical_angle)) * np.sin(np.deg2rad(self.horizontal_angle))
        z_joint2 = z_joint3 + self.forearm_length * np.cos(np.deg2rad(self.vertical_angle))

        x_joint1 = x_joint2 - self.arm_length * np.cos(np.deg2rad(self.vertical_angle)) * np.cos(np.deg2rad(self.horizontal_angle))
        y_joint1 = y_joint2 - self.arm_length * np.cos(np.deg2rad(self.vertical_angle)) * np.sin(np.deg2rad(self.horizontal_angle))
        z_joint1 = z_joint2 + self.arm_length * np.sin(np.deg2rad(self.vertical_angle))

        # apply offsets for camera position
        offsets = np.array([-15, 0, -20])
        x_joint1, y_joint1, z_joint1 = np.add([x_joint1, y_joint1, z_joint1], offsets)
        x_joint2, y_joint2, z_joint2 = np.add([x_joint2, y_joint2, z_joint2], offsets)
        x_joint3, y_joint3, z_joint3 = np.add([x_joint3, y_joint3, z_joint3], offsets)


        # calculate servo angles
        waist_angle = np.arctan2(y_joint1, x_joint1)
        shoulder_angle = np.arctan2(z_joint2 - z_joint1, np.sqrt((x_joint2 - x_joint1) ** 2 + (y_joint2 - y_joint1) ** 2))
        elbow_angle = np.arctan2(np.sqrt((x_joint2 - x_joint1) ** 2 + (y_joint2 - y_joint1) ** 2), z_joint2 - z_joint3) - np.arctan2(10.5, np.sqrt((x_joint2 - x_joint1) ** 2 + (y_joint2 - y_joint1) ** 2))
        wrist_angle = np.arctan2(z_joint3 - z_joint2, np.sqrt((x_joint3 - x_joint2) ** 2 + (y_joint3 - y_joint2) ** 2))
        gripper_angle = 0 # assume the gripper is fully closed by default

        # convert for our robot arm
        waist_commmmand = np.clip(waist_angle, -2.7, 2.7)
        shoulder_command = np.clip(shoulder_angle, -2.7, 2.7)
        elbow_command = np.clip(elbow_angle, -2.7, 2.7)
        wrist_command = np.clip(wrist_angle, -1.75, 1.75)
        gripper_command = -0.4 # assume the gripper is fully open by default
        
        return waist_commmmand, shoulder_command, elbow_command, wrist_command, gripper_command

    
    def bounding_box_angles(self):
        x_center = (self.x_max_norm + self.x_min_norm) / 2.0
        y_center = (self.y_max_norm + self.y_min_norm) / 2.0
        self.object_center_x = x_center * self.width_img
        self.object_center_y = y_center * self.height_img

        # Calculate angles
        self.horizontal_angle = (self.object_center_x - self.width_img/2.0) * self.beta/self.width_img
        self.vertical_angle = (self.height_img/2.0 - self.object_center_y) * self.alpha/self.height_img

        print("horizontal_angle: ", self.horizontal_angle)
        print("vertical_angle: ", self.vertical_angle)

    def coordinates_callback(self, coordinates_msg):
        
        x_min = coordinates_msg.x1
        y_min = coordinates_msg.y1
        x_max = coordinates_msg.x2
        y_max = coordinates_msg.y2

        # normalize coordinates
        self.x_min_norm = x_min / self.width_img * 2 - 1
        self.y_min_norm = y_min / self.height_img * 2 - 1
        self.x_max_norm = x_max / self.width_img * 2 - 1
        self.y_max_norm = y_max / self.height_img * 2 - 1

        # center coordinates of the object in image
        x_center = (self.x_max_norm + self.x_min_norm) / 2
        y_center = (self.y_max_norm + self.y_min_norm) / 2

        self.object_center_x = x_center
        self.object_center_y = y_center


    def shutdown(self):
        rospy.loginfo('Stop arm coordinates publisher.')

if __name__ == '__main__':
    calculator = CoordinatesCalculator()
    rospy.spin()