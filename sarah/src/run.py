#!/usr/bin/env python
#roslaunch rplidar_ros rplidar_s1.launch (for RPLIDAR S1)

#rosrun rplidar_ros rplidarNodeClient

#You should see rplidar's scan result in the console

#Notice: the different is serial_baudrate between A1/A2 and A3/S1

#RPLidar frame

import rospy
import time
from sensor_msgs.msg import Joy
import tf2_ros
import tf2_geometry_msgs
import subprocess

time.sleep(2)
Robot = None
NodeManager = None
	
class RunBehaviour:
	cmd_vel = None
	
	def __init__(self):
		self.start()
		
	def start(self):
		rospy.init_node('bricks',anonymous = False)
		rospy.on_shutdown(self.shutdown)
		
		
		self.tf_buffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tf_buffer)
		
		self.state_in_prog = 0
		#B button
		self.BUTTON_INDEX = 2
		self.prev_state = 0
		self.start_pose = None
		
		sub = rospy.Subscriber('joy', Joy, self.callback)
		
		rospy.loginfo("program was started")
		rospy.spin()
		
	def shutdown(self):
		# stop turtlebot
		rospy.loginfo("program was stopped")
		rospy.sleep(1)
		
	def callback(self, msg):
		state_button = msg.buttons[self.BUTTON_INDEX]
		if self.prev_state == 0 and state_button == 1:
			self.move_to_next()	
		
		self.prev_state = state_button
		
	def move_to_next(self):
		#State machine
		if self.state_in_prog == 0:
			transform = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time())
			self.start_pose = [transform.transform.translation.x, transform.transform.translation.y]
			rospy.loginfo(self.start_pose)
			
			NodeManager.start_following()
			self.state_in_prog += 1
		
		if self.state_in_prog == 1:
			NodeManager.stop_following()
			# Extend arm for grab
			self.state_in_prog += 1
		
		if self.state_in_prog == 2:
			# Store coordinates as end
			# Grab and put on body
			# Drive to start
			# Extend and drop
			# Drive to end
			# Follow
			self.state_in_prog = 1
		
class RosNodeManager:
	
	def __init__(self):
		self.following = None
		
	def start_following(self):
		if self.following is None:
			command = ['roslaunch', 'turtlebot_follower', 'follower.launch']
			self.following = subprocess.Popen(command) 	
			rospy.loginfo("Now following")
	
	def stop_following(self):
		if self.following is not None:
			self.following.terminate()
			self.following.wait()
			self.following = None
			rospy.loginfo("Stopped following")

if __name__ == '__main__':
	NodeManager = RosNodeManager()
	Robot = RunBehaviour();
	'''try:
		GoToWall()
    	except rospy.ROSInterruptException:
        	rospy.loginfo("node terminated.");'''
