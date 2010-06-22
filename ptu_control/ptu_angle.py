#!/usr/bin/env python
import roslib; roslib.load_manifest('ptu_control')
from optic_flow.msg import FlowField
from std_msgs.msg import Float64
import rospy
import numpy as np
import pylab
from math import acos, pi, degrees, atan2
from nav_msgs.msg import Odometry

def dict_to_array(d):
	data_x = []
	data_y = []
	for k,v in d.iteritems():
		for p in v:
			data_x.append(k)
			data_y.append(p)
	return np.array(data_x), np.array(data_y)

class FlowAccumulator(object):
	accum = None
	def __init__(self):
		self.FOCAL_LENGTH = 548.405693/2 #TODO get this from the camera_info message
		self.CAMERA_CENTER = 329.581393/2
		self.CENTER_COL = 150
		self.last_vel = 0.25
		
		self.x = []
		self.y = []
		self.m = 1
		self.b = 1
		
		rospy.Subscriber('/flow', FlowField, self.flow_cb)
		rospy.Subscriber('/odom', Odometry, self.odom_cb)
		self.center_pub = rospy.Publisher('/ptu_center_estimate', Float64)
		last_time = rospy.Time.now()
		# rospy.spin()
		self.viz_loop()
		
	def viz_loop(self):
		r = rospy.Rate(5)
		while not rospy.is_shutdown():
			pylab.clf()
			pylab.plot(self.x, self.y, '.')
			pylab.plot(list(set(self.x)), [self.m*v+self.b for v in set(self.x)], linewidth=4)
			pylab.axhline(y=self.m, color='g', linewidth=2)
			pylab.axvline(x=-self.b/self.m, color='g', linewidth=2)
			pylab.title('Center col: %s' % (-self.b/self.m))
			pylab.draw()
	def angle_from_col(self, col):
		return atan2(self.FOCAL_LENGTH, self.CAMERA_CENTER - col)
		
	def flow_cb(self, flow_msg):
		if self.accum == None:
			s = flow_msg.img_size[0]
			self.accum = dict(zip(range(s), [[] for i in range(s)]))
		if abs(self.last_vel) > 0.05:
			feat_a = zip(flow_msg.last_xs, flow_msg.last_ys)
			feat_b = zip(flow_msg.curr_xs, flow_msg.curr_ys)		
			flow_vectors = np.array(feat_b) - np.array(feat_a)*1.0
			for pt, v in zip(feat_a, flow_vectors):
				self.accum[pt[0]].append(v[0])
			# angle = self.angle_from_col(self.find_center()) - self.angle_from_col(self.CENTER_COL)
			# self.center_pub.publish(degrees(angle))
			self.center_pub.publish(self.find_center())
	
	def odom_cb(self, msg):
		self.last_vel = msg.twist.twist.linear.x
	
	def find_center(self):
		x, y = dict_to_array(self.accum)
		y /= self.last_vel
		m, b = np.polyfit(x, y, 1)
		self.x = x
		self.y = y
		self.m = m
		self.b = b
		return -b/m
		
if __name__ == '__main__':
	rospy.init_node('ptu_angle')
	FlowAccumulator()