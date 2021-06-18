from __future__ import division
import rospy
from geometry_msgs.msg import Twist
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
		self.net = jetson.inference.detectNet("ssd_mobilenet_v1_coco", threshold=0.7)
		self.camera = jetson.utils.videoSource("/dev/video0")
		self.display = jetson.utils.videoOutput("Human Detection.mp4")
		# Create publisher for movement commands
		self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
		# Allow the node to start
		rospy.sleep(4)
		rospy.spin()

	def process_rgb_image(self, msg):     
		im = self.camera.Capture()
		detections = self.net.Detect(im)
		self.display.Render(img)
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







