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

        # Camera parameters
        self.camera_height = -20.0 # cm
        self.camera_distance = -15.0 # cm
        self.camera_fov_horizontal = 60.0 # degrees
        self.camera_fov_vertical = 49.5 # degrees
        # Arm parameters
        self.arm_segment_lengths = [10.5, 10.5, 12.0] # cm
        self.angle_horizontal = 0
        self.angle_vertical = 0

        # load image
        img = cv2.imread('/home/mustar/jupiter/matus/matus_showcase/images/photo.png')

        # image dimensions
        self.image_width, self.image_height, self.channels = img.shape
        print(self.image_width, self.image_height)

        # get the normalized bbox coordinates from topic  ObjectCoordinates
        # sleep for 1 second to give enough time for calculation
        self.bbox = (0,0,0,0)
        rospy.Subscriber('objectCoordinates', ObjectCoordinates, self.coordinates_callback)
        rospy.sleep(1)
        print(self.bbox)
        self.joint_positions = self.calculate_joint_positions()
        print(self.joint_positions)
        self.joint_angles = self.calculate_joint_angles()
        print(self.joint_angles)
        

    def bbox_to_camera_frame(self, bbox):
        # Calculate the center of the bounding box in pixel coordinates
        bbox_center_x = bbox[0] + bbox[2] / 2.0
        bbox_center_y = bbox[1] + bbox[3] / 2.0

        # Convert pixel coordinates to normalized coordinates
        x = (2.0 * bbox_center_x / self.image_width - 1.0)
        y = (2.0 * bbox_center_y / self.image_height - 1.0)

        # Calculate the horizontal and vertical angles from the camera to the object
        self.angle_horizontal = np.deg2rad(x * self.camera_fov_horizontal / 2.0)
        self.angle_vertical = np.deg2rad(y * self.camera_fov_vertical / 2.0)

        # Calculate the distance from the camera to the object
        distance = (bbox[2] + bbox[3]) / 2.0 / (2.0 * np.tan(np.deg2rad(self.camera_fov_horizontal / 2.0)))

        # Calculate the x, y, and z coordinates of the object relative to the camera
        x = distance * np.sin(self.angle_horizontal)
        y = distance * np.sin(self.angle_vertical)
        z = distance * np.cos(self.angle_horizontal) * np.cos(self.angle_vertical)

       # Adjust the coordinates for the camera position
        x += self.camera_distance
        y += self.camera_height

        return np.array([x, y, z])
    
    def calculate_joint_positions(self):
        object_pos = self.bbox_to_camera_frame(self.bbox)
        last_joint_offset = np.array([0, 0, self.arm_segment_lengths[2]])
        
        # Calculate the position of the end-effector (grabber)
        end_effector_pos = object_pos + last_joint_offset
        print("End-effector position:", end_effector_pos)

        last_segment_length = self.arm_segment_lengths[2]
        # Calculate the position of the last joint
        last_joint_pos = end_effector_pos - last_segment_length * np.array([np.cos(self.angle_horizontal) * np.cos(self.angle_vertical), 
                                                                            np.sin(self.angle_vertical), 
                                                                            np.sin(self.angle_horizontal) * np.cos(self.angle_vertical)])
        print("Last joint position:", last_joint_pos)

        second_segment_length = self.arm_segment_lengths[1]
        # Calculate the position of the second joint
        second_joint_pos = np.array([last_joint_pos[0], 
                                     last_joint_pos[1] - second_segment_length, 
                                     last_joint_pos[2]])
        print("Second joint position:", second_joint_pos)

        first_segment_length = self.arm_segment_lengths[0]
        # Calculate the position of the first joint
        first_joint_lateral_pos = object_pos[0] - self.camera_distance
        first_joint_horizontal_pos = object_pos[2]
        first_joint_pos = np.array([first_joint_lateral_pos, 
                                    second_joint_pos[1] - first_segment_length, 
                                    first_joint_horizontal_pos])
        print("First joint position:", first_joint_pos)

        return first_joint_pos, second_joint_pos, last_joint_pos

    def calculate_joint_angles(self):
        """
        Calculates the joint angles given the positions of the three joints of the robot arm.

        Parameters:
        joint_positions (np.array): The positions of the three joints of the robot arm, in the format [[x1, y1, z1], [x2, y2, z2], [x3, y3, z3]].

        Returns:
        np.array: The joint angles in radians, in the format [theta1, theta2, theta3].
        """
        # Extract the joint positions
        first_joint_pos = self.joint_positions[0]
        second_joint_pos = self.joint_positions[1]
        last_joint_pos = self.joint_positions[2]

        # Calculate the vectors between the joints
        v1 = second_joint_pos - first_joint_pos
        v2 = last_joint_pos - second_joint_pos

        # Calculate the lengths of the vectors
        a = np.linalg.norm(v1)
        b = np.linalg.norm(v2)

        # Calculate the dot product between the vectors
        c = np.dot(v1, v2)

        # Calculate the angle between the vectors
        angle = np.arccos(c / (a * b))
        last_segment_length = self.arm_segment_lengths[2]

        # Calculate the joint angles using the law of cosines
        theta1 = np.arctan2(first_joint_pos[0], first_joint_pos[2])
        theta2 = np.pi - np.arctan2(last_joint_pos[1] - second_joint_pos[1], np.sqrt(np.power(last_joint_pos[0] - second_joint_pos[0], 2) + np.power(last_joint_pos[2] - second_joint_pos[2], 2))) - angle
        theta3 = np.pi - np.arccos((np.power(a, 2) + np.power(b, 2) - np.power(last_segment_length, 2)) / (2 * a * b))

        return np.array([theta1, theta2, theta3])


    def coordinates_callback(self, coordinates_msg):
        
        x_min = coordinates_msg.x1
        y_min = coordinates_msg.y1
        x_max = coordinates_msg.x2
        y_max = coordinates_msg.y2

        self.bbox = (x_min, y_min, x_max, y_max)


    def shutdown(self):
        rospy.loginfo('Stop arm coordinates publisher.')

if __name__ == '__main__':
    calculator = CoordinatesCalculator()
    rospy.spin()
