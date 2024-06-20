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
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from geometry_msgs.msg import PoseStamped, Quaternion

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
		#self.fake_joy = rospy.Publisher('/joy', Joy, queue_size = 10)
		
		#Turn off cause on by default.
		#self.toggle_following()
		
		self.state_in_prog = 0
		#B button
		self.BUTTON_INDEX = 2
		self.prev_state = 0
		self.start_pose = None
		self.end_pose = None
		
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
			self.start_pose = transform.transform
			rospy.loginfo(self.start_pose)
			
			#self.toggle_following()
			self.state_in_prog += 1
		
		elif self.state_in_prog == 1:
			# self.toggle_following()
			NodeManager.start_script("bring_front")
			self.state_in_prog += 1
		
		elif self.state_in_prog == 2:
			transform = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time())
			self.end_pose = transform.transform
			NodeManager.start_script("take")
			NodeManager.start_script("park")
			self.send_goal(self.start_pose)
			NodeManager.start_script("put")
			self.send_goal(self.end_pose)
			self.state_in_prog = 1
			
	def send_goal(self, pose):
		client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		client.wait_for_server()
		
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		
		goal.target_pose.pose.position.x = pose.translation.x
		goal.target_pose.pose.position.y = pose.translation.y
		
		goal.target_pose.pose.orientation = Quaternion(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w)
		
		client.send_goal(goal)
		client.wait_for_result()
		
		return client.get_result()
			
	def toggle_following(self):	
		# Figured, that the following node has a switch implemented that listens 
		# To joystick 5 for turning following on/off, which I take use off.
		joy_msg = Joy()
		
		joy_msg.axes = [0.0] * 6
		
		joy_msg.buttons = [0] * 12
		
		joy_msg.buttons[5] = 1
		
		self.fake_joy.publish(joy_msg)
		
		rospy.loginfo("Toggled following")
		
		
class RosNodeManager:
	
	def __init__(self):
		self.process = None
	
	def start_script(self, script):
		path = "~/jupiter/sarah/src/" + script + ".sh"
		process = subprocess.Popen(path, shell=True)
		process.wait()
		rospy.loginfo(script + " finished executing.")

if __name__ == '__main__':
	NodeManager = RosNodeManager()
	Robot = RunBehaviour();
	'''try:
		GoToWall()
    	except rospy.ROSInterruptException:
        	rospy.loginfo("node terminated.");'''
