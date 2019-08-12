#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_robot import MoveRobot

class LineFollower(object):

	def __init__(self):

		self.bridge_object = CvBridge()
		self.image_sub = rospy.Subscriber("/kinect/rgb/image_raw",Image,self.camera_callback)
		self.moverobot_object = MoveRobot()

	def camera_callback(self,data):

		try:
			# We select bgr8 because its the OpneCV encoding by default
			cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
		except CvBridgeError as e:
			print(e)

		
		# Crop the image
		height, width, channels = cv_image.shape
		descentre = 220
		rows_to_watch = 20
		crop_img = cv_image[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]

		# Convert from RGB to HSV
		hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

		# Define the Yellow Colour in HSV
		#RGB
		#[[[222,255,0]]]
		#BGR
		#[[[0,255,222]]]
		"""
		To know which color to track in HSV, Put in BGR. Use ColorZilla to get the color registered by the camera
		>>> yellow = np.uint8([[[B,G,R ]]])
		>>> hsv_yellow = cv2.cvtColor(yellow,cv2.COLOR_BGR2HSV)
		>>> print( hsv_yellow )
		[[[ 34 255 255]]
		"""
		lower_yellow = np.array([20,100,100])
		upper_yellow = np.array([50,255,255])

		# Threshold the HSV image to get only yellow colors
		mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
	
		# Bitwise-AND mask and original image
		res = cv2.bitwise_and(crop_img,crop_img, mask= mask)

		
		# Multiple Centres
		im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		centres = []

		for i in range(len(contours)):
			moments = cv2.moments(contours[i])

			try:
				centres.append((int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])))

			except ZeroDivisionError: 
				pass
				

		rospy.loginfo(str(centres))
		#Select the left centroid
		most_left_centroid_index = 0
		index = 0
		min_x_value = width

		for y in centres:
			# Retrieve the cx value
			cx = y[0]
			# Get the LEAST Cx value
			if cx <= min_x_value:
				min_x_value = cx
				most_left_centroid_index = index
			index += 1
		try:
			cx = centres[most_left_centroid_index][0]
			cy = centres[most_left_centroid_index][1]
			# rospy.loginfo("Min =="+str(cx)+","+str(cy)+"")
			
			cv2.circle(res,(int(cx), int(cy)), 5,(0,0,255),-1)

			error_x = cx - width / 2;
			twist_object = Twist();
			twist_object.linear.x = 0.5;
			twist_object.angular.z = -error_x / 50;
			self.moverobot_object.move_robot(twist_object)

		except: # When no line is found - Recovery Behaviour
			rospy.loginfo("Finding Target...")
			twist_object = Twist()
			twist_object.angular.z = 1
			self.moverobot_object.move_robot(twist_object)
		
		cv2.imshow("Original", cv_image)
		cv2.imshow("RES", res)
		cv2.waitKey(1)


	def clean_up(self):
		self.moverobot_object.clean_class()
		cv2.destroyAllWindows()


def main():
	rospy.init_node('line_following_node', anonymous=True)
	line_follower_object = LineFollower()
	rate = rospy.Rate(5)
	ctrl_c = False

	def shutdownhook():
	# works better than the rospy.is_shut_down()
		line_follower_object.clean_up()
		rospy.loginfo("shutdown time!")
		ctrl_c = True

	rospy.on_shutdown(shutdownhook)

	while not ctrl_c:
		rate.sleep()


if __name__ == '__main__':
	main()
