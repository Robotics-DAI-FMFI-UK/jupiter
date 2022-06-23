
import rospy

from geometry_msgs.msg import Twist
from math import radians
from sensor_msgs.msg import LaserScan
import logging
import math 

#option 1
logging.basicConfig(filename='log2.txt',filemode='a',format='%(asctime)s - %(message)s', level=logging.ERROR)

import sys;
#rospy.init_node('scan_values')
#sub = rospy.Subscriber('/scan', LaserScan, callback)
#rospy.spin()
frontwall = 100
def callback(msg):
	global frontwall
	#print('callback')
	# red buton 90
	# front 0 5 30
	# back 200
	# mike 245
	a =msg.ranges[0:0+5]
	a+= msg.ranges[355:359]
	count = 10;
	total=0;
	for i in a:
		if(not math.isinf(i)):
			total += i;
		else:
			count-=1;
	frontwall =total/count;
		

class GoToWall:
	cmd_vel=None; 
	def __init__(self):
		self.start() 

	def start(self):
		global frontwall
		rospy.init_node('forwardrun',anonymous = False)
		rospy.on_shutdown(self.shutdown)
		 
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		sub = rospy.Subscriber('/scan', LaserScan, callback)

		rospy.loginfo("program was started")
     
		# 5 HZ
        	self.r = rospy.Rate(5);
		
		#
		# let's go forward at 0.1 m/s
		self.move_cmd = Twist()
		self.move_cmd.linear.x = 0.1;
		self.move_cmd.angular.z = 0; 
		# let's go forward at 0.1 m/s
		self.stop_cmd = Twist()
		self.stop_cmd.linear.x = 0;
		self.stop_cmd.angular.z = 0;
		
		self.state = 'go to wall';
		while not rospy.is_shutdown():
			self.callbystate();
			if(self.state == 'break'):
				break;
		self.cmd_vel.publish(self.stop_cmd)
	
	def callbystate(self):
		global frontwall
		if(self.state == 'go to wall'):
			self.cmd_vel.publish(self.move_cmd)
			print(frontwall);
			if(frontwall<0.4):
				self.state= 'break';

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
