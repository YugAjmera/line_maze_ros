#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class MoveRobot(object):
	def __init__(self):
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.last_cmdvel_command = Twist()
		self._cmdvel_pub_rate = rospy.Rate(10)
		self.shutdown_detected = False

	def move_robot(self, twist_object):
		self.cmd_vel_pub.publish(twist_object)

	def clean_class(self):
		# Stop Robot
		twist_object = Twist()
		twist_object.angular.z = 0.0
		self.move_robot(twist_object)
		self.shutdown_detected = True


