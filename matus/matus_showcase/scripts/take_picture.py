#!/usr/bin/env python

from __future__ import print_function
import sys
import rospy
import cv2
import os
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

class TakePicture:
    
    def __init__(self):
        self.bridge = CvBridge()
        self.image_recieved = False
        image_topic = "/camera_top/rgb/image_raw"
        self.image_subscriber = rospy.Subscriber(image_topic, Image, self.callback)
        rospy.sleep(1)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = self.resize_image(cv_image)
        except CvBridgeError as e:
            print(e)

        self.image_recieved = True
        self.image = cv_image

    def resize_image(self, image):
        height = image.shape[0]
        width = image.shape[1]

        # We want the new image to be 60% of the original image
        scale_factor = 0.6
        new_height = int(height * scale_factor)
        new_width = int(width * scale_factor)
        dimensions = (new_width, new_height)
        new_image = cv2.resize(image, dimensions, interpolation=cv2.INTER_LINEAR)
        return new_image
    
    def take_picture(self, img_title):
        if self.image_recieved:
            path = "src/matus_showcase/images"
            cv2.imwrite(os.path.join(path, img_title), self.image)
            # cv2.imwrite(img_title, self.image)
            return True
        return False

if __name__ == '__main__':

    rospy.init_node('take_photo', anonymous=False)
    camera = TakePicture()

    timestamp = time.strftime("%Y%m%d-%H%M%S-")
    img_title = timestamp + "photo.png"

    if camera.take_picture(img_title):
        rospy.loginfo("Image " + img_title + " to /images")
    else:
        rospy.loginfo("No image recieved")

    rospy.sleep(1)