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

        self.camera_horizontal_fov = 60
        self.camera_vertical_fov = 49.5

        # image dimensions
        self.image_width, self.image_height, self.channels = img.shape
        print("Image dimensions: ",self.image_width, self.image_height)
        self.min_waist_angle = -0.785
        self.max_waist_angle = 0.785

        self.bbox = (0,0,0,0)
        self.center_x = 0
        self.center_y = 0
        self.waist_angle = 0
       
        self.waist_angle_pub = rospy.Publisher('waistAngle', Float64, queue_size=10)
        rospy.Subscriber('objectCoordinates', ObjectCoordinates, self.coordinates_callback)

        rospy.sleep(1)

    def publish_waist_angle(self):
        waist_angle = self.calculate_waist_turn()
        
        msg = Float64()
        msg.data = waist_angle
        print("TURN ",msg.data)
        self.waist_angle_pub.publish(msg)

    def calculate_waist_turn(self):

        # radians_range = self.max_waist_angle - (self.min_waist_angle)
        # image_width = self.image_width
        # center_pixel = self.center_x
        
        # radians = (center_pixel / image_width) * radians_range + (self.min_waist_angle)
        # return -radians

        # Step 2: Calculate bbox center in image coordinates
        alpha_h = (self.camera_horizontal_fov * np.pi/180) / 640
        alpha_v = (self.camera_vertical_fov * np.pi/180) / 480
        
        # Step 3: Convert bbox center coordinates to camera coordinate system
        cx_cam = self.center_x - 15/100
        cy_cam = self.center_y -20/100

        # Step 4: Calculate angles of bbox center point in camera coordinate system
        theta_h = alpha_h * (cx_cam - (640 / 2))
        theta_v = alpha_v * (cy_cam - (480 / 2))

        self.waist_angle = np.arctan2(theta_v, theta_h)

        return self.waist_angle

    def calculate_object_distance_from_arm(self):
        
        bbox_width = self.bbox[2] - self.bbox[0] 
        bbox_height = self.bbox[3] - self.bbox[1]

        half_width = bbox_width / 2
        half_height = bbox_height / 2

        object_width = 0.07 #in m
        object_height = 0.08 #in m

        distance_horizontal = (object_width/ 2) / np.tan((60 * np.pi/180) / 2) / half_width
        distance_vertical = (object_height / 2) / np.tan((49.5 * np.pi/180) / 2) / half_height

        distance = (distance_horizontal + distance_vertical) / 2

        distance_to_arm = np.sqrt((distance - 0.20)**2 + (distance - 0.15)**2)
        
        return distance_to_arm


    def calculate_joint_positions(self):
        # Joint distances
        L1 = 10.5 / 100  # Convert cm to m
        L2 = 10.5 / 100  # Convert cm to m
        L3 = 12 / 100    # Convert cm to m

        # End effector position (grabber)
        d = self.calculate_object_distance_from_arm()
        x_grabber = d * np.cos(self.waist_angle)
        y_grabber = d * np.sin(self.waist_angle)

        # Joint 1
        theta1 = np.arctan2(y_grabber, x_grabber)

        # Joint 2
        x2 = L1 * np.cos(theta1)
        y2 = L1 * np.sin(theta1)

        # Joint 3
        theta2 = np.arctan2(y_grabber - y2, x_grabber - x2)
        x3 = x2 + L2 * np.cos(theta1 + theta2)
        y3 = y2 + L2 * np.sin(theta1 + theta2)

        # Joint 3 to grabber (End effector)
        theta3 = np.arctan2(y_grabber - y3, x_grabber - x3)

        # Convert angles to radians
        theta1 = np.radians(theta1)
        theta2 = np.radians(theta2)
        theta3 = np.radians(theta3)

        # Print the results
        print("Joint 1 angle:", theta1)
        print("Joint 2 angle:", theta2)
        print("Joint 3 angle:", theta3)
    
    def coordinates_callback(self, coordinates_msg):
        
        x_min = coordinates_msg.x1
        y_min = coordinates_msg.y1
        x_max = coordinates_msg.x2
        y_max = coordinates_msg.y2

        self.bbox = (x_min, y_min, x_max, y_max)

        self.center_x = (x_min + x_max) / 2
        self.center_y = (y_min + y_max) / 2

        self.waist_angle = self.calculate_waist_turn()

        print(self.center_x)
        
        self.publish_waist_angle()

        rospy.signal_shutdown("Shuting Down")

    def shutdown(self):
        rospy.loginfo('Stop arm coordinates publisher.')

if __name__ == '__main__':
    calculator = CoordinatesCalculator()
    rospy.spin()
