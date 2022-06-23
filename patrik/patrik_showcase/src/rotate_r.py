
import rospy

from geometry_msgs.msg import Twist
from math import radians
import logging
import math 

#option 1
logging.basicConfig(filename='log2.txt',filemode='a',format='%(asctime)s - %(message)s', level=logging.ERROR)

import sys;
#rospy.init_node('scan_values')
#sub = rospy.Subscriber('/scan', LaserScan, callback)
#rospy.spin()
		

class GoToWall:
	cmd_vel=None; 
	def __init__(self):
		self.start() 

	def start(self):
		rospy.init_node('forwardrun',anonymous = False)
		rospy.on_shutdown(self.shutdown)
		 
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

		rospy.loginfo("program was started")
     
		# 5 HZ
        	self.r = rospy.Rate(5);
		
		#
		# let's go forward at 0.1 m/s
		self.move_cmd = Twist()
		self.move_cmd.linear.x = 0;
		self.move_cmd.angular.z = 0.5; 
		# let's go forward at 0.1 m/s
		self.stop_cmd = Twist()
		self.stop_cmd.linear.x = 0;
		self.stop_cmd.angular.z = 0;
		
		self.state = 'go to wall';
		while not rospy.is_shutdown():
			self.cmd_vel.publish(self.move_cmd)
	
	

	def checkIfWallisFar(self):
		#detect wall
		return True;

	def shutdown(self):
       		# stop turtlebot
        	rospy.loginfo("program was stopwed")
        	self.cmd_vel.publish(Twist())
        	rospy.sleep(1)


if __name__ == '__main__':
	try:
		GoToWall()
    	except rospy.ROSInterruptException:
        	rospy.loginfo("node terminated.");
