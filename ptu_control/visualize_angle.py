#!/usr/bin/env python
import roslib; roslib.load_manifest('ptu_control')
import rospy
import cv
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class AngleViz(object):
	last_angle = 0
	bridge = CvBridge()

	def __init__(self):
		cv.NamedWindow('win')
		rospy.Subscriber('/image', Image, self.image_cb)
		rospy.Subscriber('/ptu_center_estimate', Float64, self.center_cb)
		rospy.spin()

	def image_cb(self, msg):
		img = self.bridge.imgmsg_to_cv(msg, "passthrough")
		cv.Line(img, (self.last_angle, 0), (self.last_angle, 239), (255,0,0))
		cv.Line(img, (160, 0), (160, 239), (0,255,0))
		
		cv.ShowImage('win', img)
		cv.WaitKey(10)
	
	def center_cb(self, msg):
		self.last_angle = msg.data
		print self.last_angle

if __name__ == '__main__':
	rospy.init_node('angleviz')
	AngleViz()