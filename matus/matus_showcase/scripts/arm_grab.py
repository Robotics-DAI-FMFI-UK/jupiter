#!/usr/bin/env python
import rospy
import numpy
from std_msgs.msg import Float64

class ArmGrab:
    def __init__(self):

        rospy.on_shutdown(self.shutdown)

        self.set_up_joint_publishers()

        self.waist_angle = 0

        self.waist_angle_subscriber = rospy.Subscriber('waistAngle', Float64, self.waist_angle_callback)

        rospy.spin()

    def waist_angle_callback(self, msg):
        self.waist_angle = msg.data
        print(msg.data)
        # Arm movement starts when waist angle is published
        self.initial_position()
        self.cup_position(self.waist_angle)
        self.put_away_position()

        rospy.signal_shutdown("Finished arm movement")  # Signal shutdown after arm movement is complete

    def initial_position(self):
        self.pos1 = 0.0
        self.pos2 = 0.0
        self.pos3 = 0.0
        self.pos4 = 0.0
        self.pos5 = 0.0
        self.waist.publish(self.pos1)
        self.shoulder.publish(self.pos2)
        self.elbow.publish(self.pos3)
        self.wrist.publish(self.pos4)
        self.hand.publish(self.pos5)

    def cup_position(self, waist_angle):
        self.pos1 = waist_angle
        self.pos2 = 1.3
        self.pos3 = 0.9
        self.pos4 = 1
        self.pos5 = -0.4
        self.hand.publish(self.pos5)
        rospy.sleep(2)
        self.wrist.publish(self.pos4)
        rospy.sleep(2)
        self.waist.publish(self.pos1)
        rospy.sleep(2)
        self.shoulder.publish(self.pos2)
        rospy.sleep(2)
        self.elbow.publish(self.pos3)
        rospy.sleep(2)
        self.wrist.publish(self.pos4)
        rospy.sleep(2)

    def put_away_position(self):
        self.pos5 = 0.25
        self.hand.publish(self.pos5)
        rospy.sleep(2)
        self.pos2 = -0.4
        self.shoulder.publish(self.pos2)
        rospy.sleep(1.5)
        self.pos5 = 0.2
        self.hand.publish(self.pos5)
        self.pos1 = -1.8
        self.waist.publish(self.pos1)
        self.pos3 = 1.8
        self.elbow.publish(self.pos3)
        self.pos4 = 1.3
        self.wrist.publish(self.pos4)
        rospy.sleep(1)
        self.pos1 = -2
        self.waist.publish(self.pos1)
        self.pos2 = 0
        self.shoulder.publish(self.pos2)
        self.pos4 = 1.3
        self.wrist.publish(self.pos4)
        self.pos5 = 0
        self.hand.publish(self.pos5)

    def set_up_joint_publishers(self):

        # publish command message to joints/servos of arm
        # max 2.7 or -2.7 and 1.5 = 90 degrees 0.75 = 45 degrees
        self.waist = rospy.Publisher('/waist_controller/command',Float64, queue_size=10)
        # max 2.7 or -2.7 and 1.5 = 90 degrees 0.75 = 45 degrees
        self.shoulder = rospy.Publisher('/shoulder_controller/command',Float64, queue_size=10)
        # max 2.7 or -2.7 and 1.5 = 90 degrees 0.75 = 45 degrees
        self.elbow = rospy.Publisher('/elbow_controller/command',Float64, queue_size=10)
        # max 1.75 or -1.75 and 1.5 = 90 degrees 0.75 = 45 degrees
        self.wrist = rospy.Publisher('/wrist_controller/command',Float64, queue_size=10)
        # max 0.7 or -0.4 and -0.4 = open wide degrees 0.7 = closed
        self.hand = rospy.Publisher('/hand_controller/command',Float64, queue_size=10)
        self.pos1 = Float64()
        self.pos2 = Float64()
        self.pos3 = Float64()
        self.pos4 = Float64()
        self.pos5 = Float64()

    def shutdown(self):
        rospy.loginfo("Shutting down robot arm....")

if __name__=="__main__":
    rospy.init_node('arm_grab')
    try:
        ArmGrab()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
