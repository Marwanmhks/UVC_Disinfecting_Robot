#! /usr/bin/env python3
from __future__ import division
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import random
import threading

class AutonavNode(object):
	def __init__(self):

		self.min_distance = 400 
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
		self.th = 0.0
		self.speed = rospy.get_param("~speed", 0.18)
		self.avoiding = False
		rospy.init_node("autonav_node")
		self.cv_bridge = CvBridge()
		# Create subscriber for Kinect depthg image
		self.sub = rospy.Subscriber("camera/depth/image_raw", Image, self.process_depth_image)
		# Create publisher for movement commands
		self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
		# Allow the node to start
		rospy.sleep(4)
		rospy.spin()

	def process_depth_image(self, msg):
		im = self.cv_bridge.imgmsg_to_cv2(msg)
		min_point = im[im.nonzero()].min()
		print(min_point)
		if min_point < self.min_distance:
			if not self.avoiding:
				threading.Thread(target=self.avoid_obstacle).start()
			else:
				self.avoiding = False

	def avoid_obstacle(self):
		print("AVOIDING")
		self.avoiding = True
		self.turning
		self.publisher.publish(Twist)
		while self.avoiding:
			rospy.sleep(1/30)
			self.straight
			self.publisher.publish(Twist)
			print("DONE AVOIDING")
	def turning(self):
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
		self.th = 1.0
	def straight(self):
		self.x = self.speed
		self.y = 0.0
		self.z = 0.0
		self.th = 0.0

if __name__ == '__main__':
	autonav_node = AutonavNode()

