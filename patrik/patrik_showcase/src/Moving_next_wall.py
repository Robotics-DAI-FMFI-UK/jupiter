#roslaunch rplidar_ros rplidar_s1.launch (for RPLIDAR S1)

#rosrun rplidar_ros rplidarNodeClient

#You should see rplidar's scan result in the console

#Notice: the different is serial_baudrate between A1/A2 and A3/S1

#RPLidar frame

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
leftwall = 100
rightwall = 100;
leftdiagonal = 100;
backwall = 100;
Robot = None;
def callback(msg):
	global frontwall,leftwall,rightwall,backwall,leftdiagonal,rightdiagonal
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
	
	#### left wall
	a =msg.ranges[265:275]
	count = 10;
	total=0;
	for i in a:
		if(not math.isinf(i)):
			total += i;
		else:
			count-=1;
	leftwall =total/count;

	#### right wall
	a =msg.ranges[85:95]
	count = 10;
	total=0;
	for i in a:
		if(not math.isinf(i)):
			total += i;
		else:
			count-=1;
	rightwall =total/count;

	#### back wall
	a =msg.ranges[175:185]
	count = 10;
	total=0;
	for i in a:
		if(not math.isinf(i)):
			total += i;
		else:
			count-=1;
	backwall =total/count;
	#### left diagonal wall
	a =msg.ranges[310:320]
	count = 10;
	total=0;
	for i in a:
		if(not math.isinf(i)):
			total += i;
		else:
			count-=1;
	leftdiagonal =total/count;
	#### right diagonal wall
	a =msg.ranges[40:50]
	count = 10;
	total=0;
	for i in a:
		if(not math.isinf(i)):
			total += i;
		else:
			count-=1;
	rightdiagonal =total/count;
	if(Robot is not None):
		pass;
		#Robot.setvalues();
		

class GoToWall:
	cmd_vel=None; 
	def __init__(self):
		self.start() 

	def start(self):
		global frontwall,leftwall,rightwall,backwall,leftdiagonal,rightdiagonal;
		rospy.init_node('nextwallrun',anonymous = False)
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
		#let's turn at 45 deg/s
		self.turnR_cmd = Twist()
		self.turnR_cmd.linear.x = 0.1
		self.turnR_cmd.angular.z = radians(45); #45 deg/s in radians/s
	 	# let's go forward at 0.2 m/s
		#let's turn at 45 deg/s
		self.turnL_cmd = Twist()
		self.turnL_cmd.linear.x = 0
		self.turnL_cmd.angular.z = -radians(45); #45 deg/s in radians/s
	 	# let's go forward at 0.2 m/s
		self.moveR_cmd = Twist()
		self.moveR_cmd.linear.x = 0.1
		self.moveR_cmd.angular.z = radians(15); #45 deg/s in radians/s
	 	# let's go forward at 0.2 m/s
		#let's turn at 45 deg/s
		self.moveL_cmd = Twist()
		self.moveL_cmd.linear.x = 0.1
		self.moveL_cmd.angular.z = -radians(15); #45 deg/s in radians/s
	 	# let's go forward at 0.2 m/s

		#go forward while hit wall
		rospy.sleep(1)
		
		if(rightwall <0.6):
			self.state = 'follow the wall';
			while not rospy.is_shutdown():
				self.callbystate2();
			return
		self.state = 'go to wall';
		if(leftwall <0.6):
			self.state= 'follow the wall';
		while not rospy.is_shutdown():
			self.callbystate();

	
	def callbystate(self):
		global frontwall,leftwall,rightwall,backwall,leftdiagonal;
		if(self.state == 'go to wall'):
			self.cmd_vel.publish(self.move_cmd)
			print(frontwall);
			if(frontwall<0.4):
				self.state= 'align with a wall'
		elif(self.state == 'align with a wall'):
			self.cmd_vel.publish(self.turnL_cmd)
			print(leftwall)
			if(leftwall<0.42):
				self.state= 'follow the wall'

		elif(self.state=='follow the wall'):
			if(frontwall<0.4):
				self.state= 'wall in front'
			if(leftwall>0.8):
				self.state= 'free on left'
			if(leftdiagonal<0.6):
				self.cmd_vel.publish(self.moveL_cmd)
			elif(leftdiagonal>0.6):
				self.cmd_vel.publish(self.moveR_cmd)
			else:
				self.cmd_vel.publish(self.move_cmd)
		elif(self.state == 'wall in front'):
			self.cmd_vel.publish(self.turnL_cmd)
			print(leftdiagonal)
			if(leftdiagonal>0.5):
				self.state= 'follow the wall'
		elif(self.state == 'free on left'):
			self.cmd_vel.publish(self.turnR_cmd)
			if(leftdiagonal < 0.7):
				self.state= 'follow the wall'
		rospy.sleep(1/5);
	
	def callbystate2(self):
		global frontwall,leftwall,rightwall,backwall,leftdiagonal;
		if(self.state == 'go to wall'):
			self.cmd_vel.publish(self.move_cmd)
			print(frontwall);
			if(frontwall<0.4):
				self.state= 'align with a wall'
		elif(self.state == 'align with a wall'):
			self.cmd_vel.publish(self.turnR_cmd)
			print(leftwall)
			if(rightwall<0.42):
				self.state= 'follow the wall'

		elif(self.state=='follow the wall'):
			if(frontwall<0.4):
				self.state= 'wall in front'
			if(rightwall>0.8):
				self.state= 'free on right'
			if(rightdiagonal<0.6):
				self.cmd_vel.publish(self.moveR_cmd)
			elif(rightdiagonal>0.6):
				self.cmd_vel.publish(self.moveL_cmd)
			else:
				self.cmd_vel.publish(self.move_cmd)
		elif(self.state == 'wall in front'):
			self.cmd_vel.publish(self.turnR_cmd)
			print(rightdiagonal)
			if(rightdiagonal>0.5):
				self.state= 'follow the wall'
		elif(self.state == 'free on right'):
			self.cmd_vel.publish(self.turnL_cmd)
			if(rightdiagonal < 0.7):
				self.state= 'follow the wall'
		rospy.sleep(1/5);

	def setvalues(self):
		pass;

	def checkIfWallisFar(self):
		#detect wall
		return True;

	def shutdown(self):
       		# stop turtlebot
        	rospy.loginfo("program was stopwed")
        	self.cmd_vel.publish(Twist())
        	rospy.sleep(1)


if __name__ == '__main__':
	Robot = GoToWall();
	'''try:
		GoToWall()
    	except rospy.ROSInterruptException:
        	rospy.loginfo("node terminated.");'''
