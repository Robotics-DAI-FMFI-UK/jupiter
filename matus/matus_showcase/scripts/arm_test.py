#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

'''
    real world dimensions:
    width = 40cm
    height = 31 cm
    depth = 32 cm
    cup_center = 
    

    image
    width = 640 pixel
    height = 480 pixel
    cup_center = 
'''

class Loop:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        # publish command message to joints/servos of arm
        # max 2.7 or -2.7 and 1.5 = 90 degrees 0.75 = 45 degrees
        self.joint1 = rospy.Publisher('/waist_controller/command',Float64)
        # max 2.7 or -2.7 and 1.5 = 90 degrees 0.75 = 45 degrees
        self.joint2 = rospy.Publisher('/shoulder_controller/command',Float64)
        # max 2.7 or -2.7 and 1.5 = 90 degrees 0.75 = 45 degrees
        self.joint3 = rospy.Publisher('/elbow_controller/command',Float64)
        # max 1.75 or -1.75 and 1.5 = 90 degrees 0.75 = 45 degrees
        self.joint4 = rospy.Publisher('/wrist_controller/command',Float64)
        # max 0.7 or -0.4 and -0.4 = open wide degrees 0.7 = closed
        self.joint5 = rospy.Publisher('/hand_controller/command',Float64)
        self.pos1 = Float64()
        self.pos2 = Float64()
        self.pos3 = Float64()
        self.pos4 = Float64()
        self.pos5 = Float64()

        while not rospy.is_shutdown():
            self.pos1 = 0.0
            self.pos2 = 0.0
            self.pos3 = 0.0
            self.pos4 = 0.0
            self.pos5 = 0.0
            self.joint1.publish(self.pos1)
            self.joint2.publish(self.pos2)
            self.joint3.publish(self.pos3)
            self.joint4.publish(self.pos4)
            self.joint5.publish(self.pos5)
            rospy.sleep(2)

            self.pos1 = 1.5
            self.pos2 = 0.0
            self.pos3 = 0.0
            self.pos4 = 0.0
            self.pos5 = 0.0
            self.joint1.publish(self.pos1)
            self.joint2.publish(self.pos2)
            self.joint3.publish(self.pos3)
            self.joint4.publish(self.pos4)
            self.joint5.publish(self.pos5)
            rospy.sleep(1)


    def cleanup(self):
        rospy.loginfo("Shutting down robot arm....")

if __name__=="__main__":
    rospy.init_node('arm')
    try:
        Loop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
