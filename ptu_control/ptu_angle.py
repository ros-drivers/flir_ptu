#!/usr/bin/env python
import roslib; roslib.load_manifest('ptu_control')
from optic_flow.msg import FlowField
from std_msgs.msg import Float64
import rospy
import numpy as np
import pylab
from math import acos, pi, degrees

class FlowAccumulator(object):
	accum = None
	def __init__(self):
		self.FOV = pi/3
		self.CENTER_COL = 300
		
		rospy.Subscriber('/flow', FlowField, self.flow_cb)
		self.center_pub = rospy.Publisher('/ptu_center_estimate', Float64)
		rospy.spin()
		
	def angle_from_col(self, col):
		return self.FOV * col / float(len(self.accum))
		
	def flow_cb(self, flow_msg):
		if self.accum == None:
			self.accum = np.zeros(flow_msg.img_size[0])
		feat_a = zip(flow_msg.last_xs, flow_msg.last_ys)
		feat_b = zip(flow_msg.curr_xs, flow_msg.curr_ys)		
		flow_vectors = np.array(feat_b) - np.array(feat_a)
		for pt, v in zip(feat_a, flow_vectors):
			self.accum[pt[0]] = (self.accum[pt[0]] + v[0])/2
		angle = self.angle_from_col(self.find_center()) - self.angle_from_col(self.CENTER_COL)
		self.center_pub.publish(degrees(angle))
	
	def find_center(self):
		mu = np.mean(self.accum)
		sigma = np.std(self.accum)

		meanmat = [mu]*self.accum.size
		sigmat = [sigma]*self.accum.size
		inliers =  np.absolute(self.accum - meanmat) < 3*sigma

		self.accum = inliers * self.accum

		filtered = np.array(filter(lambda x: x != 0, self.accum))
		m, b = np.polyfit(range(filtered.size), filtered, 1)
		return -b/m
		
if __name__ == '__main__':
	rospy.init_node('ptu_angle')
	FlowAccumulator()