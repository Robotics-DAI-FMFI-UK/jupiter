#roslaunch rplidar_ros rplidar_s1.launch (for RPLIDAR S1)

#rosrun rplidar_ros rplidarNodeClient

#You should see rplidar's scan result in the console

#Notice: the different is serial_baudrate between A1/A2 and A3/S1

#RPLidar frame

import rospy

from geometry_msgs.msg import Twist
from math import radians
from sensor_msgs.msg import LaserScan
#from moveit_msgs import GetStateValidityRequest, GetStateValidity
#from moveit_msgs.msg import RobotState
import logging
import math 

#option 1
logging.basicConfig(filename='log2.txt',filemode='a',format='%(asctime)s - %(message)s', level=logging.ERROR)

import sys;
#rospy.init_node('scan_values')
#sub = rospy.Subscriber('/scan', LaserScan, callback)
#rospy.spin()
frontwall = 100;
leftdiagonal = 100;
rightdiagonal = 100;
Robot = None;
def callback(msg):
	
#	self.rs = RobotState()
#	self.rs.joint_state.name = ['joint1','joint2']
#	self.rs.joint_state.position = [0.0,0.0]
#	self.joint_state_received = False
	
	global frontwall,leftdiagonal,rightdiagonal, left, right
	#### front wall
	a =msg.ranges[0:0+5]
	a+= msg.ranges[355:359]
	count = 10;
	total=0;
	for i in a:
		if(not math.isinf(i)):
			total += i;
		else:
			count-=1;
	if count != 0:
		frontwall =total/count;
	else:
		frontwall = 0;
	
	#### left diagonal wall
	a =msg.ranges[300:340]
	count = 40;
	total=0;
	for i in a:
		if(not math.isinf(i)):
			total += i;
		else:
			count-=1;
	if count != 0:
		leftdiagonal =total/count;
	else:
		leftdiagonal = 0;
	
	#### right diagonal wall
	a =msg.ranges[20:60]
	count = 40;
	total=0;
	for i in a:
		if(not math.isinf(i)):
			total += i;
		else:
			count-=1;
	if count != 0:
		rightdiagonal =total/count;
	else:
		rightdiagonal = 0;
	
	#### left wall
	a =msg.ranges[260:280]
	count = 20;
	total=0;
	for i in a:
		if(not math.isinf(i)):
			total += i;
		else:
			count-=1;
	if count != 0:
		left =total/count;
	else:
		left = 0;
	
	#### right wall
	a =msg.ranges[80:100]
	count = 20;
	total=0;
	for i in a:
		if(not math.isinf(i)):
			total += i;
		else:
			count-=1;
	if count != 0:
		right =total/count;
	else:
		right = 0;

class GoToWall:
	cmd_vel=None; 
	def __init__(self):
		self.start() 

	def start(self):
		counter = 0
		global frontwall,leftdiagonal,rightdiagonal, left, right, counter;
		rospy.init_node('nextwallrun',anonymous = False)
		rospy.on_shutdown(self.shutdown)
		 
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		sub = rospy.Subscriber('/scan', LaserScan, callback)

		rospy.loginfo("program was started")
     
		# 5 HZ
		self.r = rospy.Rate(5)
		
		#
		# let's go forward at 0.3 m/s
		self.move_cmd = Twist()
		self.move_cmd.linear.x = 0.25
		self.move_cmd.angular.z = 0
		# let's turn at 10 deg/s
		self.turnL_cmd = Twist()
		self.turnL_cmd.linear.x = 0.25
		self.turnL_cmd.angular.z = radians(20)
		# let's turn at 10 deg/s
		self.turnR_cmd = Twist()
		self.turnR_cmd.linear.x = 0.25
		self.turnR_cmd.angular.z = -radians(20)
		# let's new right way
		self.newR = Twist()
		self.newR.linear.x = 0
		self.newR.angular.z = -radians(45)
		# let's new left way
		self.newL = Twist()
		self.newL.linear.x = 0
		self.newL.angular.z = radians(45)

		#go forward while hit wall
		rospy.sleep(1)
		
		self.state = 'go';
		while not rospy.is_shutdown():
			self.callbystate()
			
#	def GetStateValidity(self, group_name='acrobat', constraints=None):
#		gsvr = GetStateValidityRequest()
#		gsvr.robot_state = self.rs
#		gsvr.group_name = group_name
#		if constriants != None:
#			gsvr.constriants = constriants
#		result = self.sr_srv.call(gsvr)
#		return result

	
	def callbystate(self):
		global frontwall, leftdiagonal, rightdiagonal, left, right, counter;
		print('{0}, f:{1:2.2f}, l:{2:2.2f}, ld:{3:2.2f}, rd:{4:2.2f}, r:{4:2.2f}'.format(self.state, frontwall, left, leftdiagonal, rightdiagonal, right))
#		print(GetStateValidity())
		if(self.state == 'go'):
			self.cmd_vel.publish(self.move_cmd)
			if(leftdiagonal <1):
				self.state = 'small right';
			elif(rightdiagonal <1):
				self.state= 'small left';
			elif(0.2<frontwall<0.6 and right > 3):
				self.state= 'right';
			elif(0.2<frontwall<0.6 and left > 3):
				self.state= 'left';
			elif(left>4):
				self.state= 'left on end'
			#elif(right>5):
				#self.state= 'right on end';
			
				
		elif(self.state == 'small left'):
			self.cmd_vel.publish(self.turnL_cmd)
			if(rightdiagonal>1):
				self.state= 'go'
				
		elif(self.state == 'small right'):
			self.cmd_vel.publish(self.turnR_cmd)
			if(leftdiagonal>1):
				self.state= 'go'
				
		elif(self.state=='right'):
			self.cmd_vel.publish(self.newR)
			if(left<0.6 and frontwall > 1):
				self.state = 'go'
		
		elif(self.state=='left'):
			self.cmd_vel.publish(self.newL)
			if(right<0.6 and frontwall > 1):
				self.state = 'go'
				
		elif(self.state=='right on end'):
			counter += 1
			if counter > 2:
				self.cmd_vel.publish(self.newR)
			if counter > 15:
				self.state = 'go'
				counter = 0
				
		elif(self.state=='left on end'):
			counter += 1
			if counter > 2:
				self.cmd_vel.publish(self.newL)
			if counter > 15:
				self.state = 'go'
				counter = 0
			
		rospy.sleep(0.15)

	def setvalues(self):
		pass;

	def checkIfWallisFar(self):
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
