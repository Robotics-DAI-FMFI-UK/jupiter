#!/usr/bin/env python

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
 
class ImageConverter:

    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2", Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.cv_image = None
 
    def callback(self,data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        cv2.imshow("Image window", self.cv_image)
        cv2.waitKey(3)
 
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)
 
def main(args):
    ic = ImageConverter()
    rospy.init_node('image_converter', anonymous=True)
    rospy.spin()
    print("Shutting down")
    cv2.destroyAllWindows()
 
if __name__ == '__main__':
     main(sys.argv)
