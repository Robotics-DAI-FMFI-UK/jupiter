#!/usr/bin/env python

from __future__ import print_function
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class TakePicture:


    # Choose which camera to take a picture with
    # Choose a path to save the picture and a name of the picture
    # Initialize a camera topic subscriber
    # top camera is default
    # run like this to change to lower camera for find_a_cup.py:
    # rosrun matus_showcase take_picture.py _camera_topic:=/camera/rgb/image_raw

    def __init__(self, camera_topic="/camera_top/rgb/image_raw"):

        self.bridge = CvBridge()
        self.image_recieved = False

        self.image_subscriber = rospy.Subscriber(camera_topic, Image, self.img_callback)
        self.img_path = "/home/mustar/jupiter/matus/matus_showcase/images/photo.png"
        rospy.sleep(1)
    
    # Read image data from camera topic and transform it to opencv format
    # Resize the image if needed
    def img_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # cv_image = self.resize_image(cv_image)
        except CvBridgeError as e:
            print(e)

        self.image_recieved = True
        self.image = cv_image

    # Change the dimensions of the image according to scale_factor
    # Scale of 0.6 changes the dimensions to 60% of the original image
    def resize_image(self, image, scale=0.6):
        height = image.shape[0]
        width = image.shape[1]

        new_height = int(height * scale)
        new_width = int(width * scale)
        dimensions = (new_width, new_height)

        new_image = cv2.resize(image, dimensions, interpolation=cv2.INTER_LINEAR)
        return new_image
    
    def save_picture(self, img_path):
        if self.image_recieved:
            cv2.imwrite(img_path, self.image)
            return True
        return False

    def take_picture(self):
        rospy.init_node('take_photo', anonymous=False)
        camera = TakePicture()


        if camera.save_picture(self.img_path):
            rospy.loginfo("Image saved to " + self.img_path)
        else:
            rospy.loginfo("No image recieved")

        rospy.sleep(1)

try:
    camera = TakePicture()
    camera.take_picture()
except Exception as e:
        print(e)