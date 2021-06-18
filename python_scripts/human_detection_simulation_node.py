from __future__ import division
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import random
import threading
import jetson.inference
import jetson.utils
	

class HumanDetectionNode(object):
	def __init__(self):
 
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
		self.th = 0.0
		self.turn = rospy.get_param("~turn", 1.0)
		self.speed = rospy.get_param("~speed", 0.5)
		self.detecting = False
		rospy.init_node("human_detection_node")
		self.cv_bridge = CvBridge()
		# Create subscriber for Kinect rgb image
		self.sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.process_rgb_image)
		# Create publisher for movement commands
		self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
		# Allow the node to start
		rospy.sleep(4)
		rospy.spin()

	def process_rgb_image(self, msg):
		net = jetson.inference.detectNet("ssd_mobilenet_v1_coco", threshold=0.7)     
		im = self.cv_bridge.imgmsg_to_cv2(msg)
		detections = net.Detect(im)
		person_detected=detections[0].ClassID
		print(person_detected)
		if person_detected==1:
		if not self.detecting:
		threading.Thread(target=self.wait_human).start()
		else:
		self.detecting = False

	def wait_human(self):
		print("STOPPING")
		self.detecting = True
		self.stop
		self.publisher.publish(twist)
		while self.detecting:
		rospy.sleep(1/30)
		self.back_on_track
		self.publisher.publish(twist)
		print("BACK ONLINE")
	def stop(self):
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
		self.th = 0.0
	def back_on_track(self):
		self.x = self.speed
		self.y = 0.0
		self.z = 0.0
		self.th = self.turn

if __name__ == '__main__':
	human_detection_node = HumanDetectionNode()







