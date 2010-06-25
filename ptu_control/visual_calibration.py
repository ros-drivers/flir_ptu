#!/usr/bin/env python
import roslib; roslib.load_manifest('ptu_control')
import rospy
import ptu_control.msg
import actionlib
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from scipy.io import savemat
from random import randint
from math import atan2
import cv
import numpy as np

from logitech_pantilt.msg import PanTilt
# from ptu_control.Calibration import pantiltReset

def FindChessboardCenter(corners):
	return np.average(corners, axis=0)

class VisualCalibration(object):
	client = actionlib.SimpleActionClient('SetPTUState', ptu_control.msg.PtuGotoAction)
	reset_client = actionlib.SimpleActionClient('ResetPtu', ptu_control.msg.PtuResetAction)
	bridge = CvBridge()
	
	last_img = None
	data = {'cb_centers':[], 'actions':[], 'ground_truth': []}
	last_pt = np.array([0,0])
	
	F_X = 548.405693/2
	F_Y = 547.708933/2
	# C_X = 329.581393/2	
	# C_Y = 239.581617/2
	C_X = 320/2
	C_Y = 240/2
	
	def __init__(self):
		self.ground_truth_pub = rospy.Publisher('ground_truth_pantilt', PanTilt)
		self.last_img_time = rospy.Time.now()
		#cv.NamedWindow('win')
		image_sub = rospy.Subscriber('image', Image, self.image_cb)
		self.client.wait_for_server()
		self.reset_client.wait_for_server()
		
		self.reset_client.send_goal(ptu_control.msg.PtuResetGoal())
		self.reset_client.wait_for_goal_to_finish()
		# wait for images to start coming in
		while not self.last_img and not rospy.is_shutdown():
			rospy.sleep(0.1)
		
		PAN_RANGE  = 10
		TILT_RANGE = 10
		
		corners = cv.FindChessboardCorners(self.last_img, (8,6))
		cb_center = FindChessboardCenter(corners[1])
		orig_cb_center = cb_center
		print cb_center
		cv.DrawChessboardCorners(self.last_img, (8,6), corners[1], corners[0])
		self.offset = self.angle_from_cb_center(cb_center)
		#cv.ShowImage('win', self.last_img)
		#cv.WaitKey(10)
		print self.offset
		# import sys; sys.exit()
		
		
		# while not rospy.is_shutdown():
		try:
			recenter_ct = 0
			for trial in range(10000):
				if rospy.is_shutdown(): break
				if 2 < recenter_ct < 4 :
					pan = 0
					tilt = 0
				else:
					pan = randint(-PAN_RANGE,PAN_RANGE)
					tilt = randint(-TILT_RANGE,TILT_RANGE)
			
				goal = ptu_control.msg.PtuGotoGoal(pan=pan, tilt=tilt)
				self.client.send_goal(goal)
				# self.client.wait_for_goal_to_finish()
				self.client.wait_for_result()
				finish_time = rospy.Time.now()
				rospy.sleep(1.0)
			
				while self.last_img_time < finish_time:
					rospy.sleep(0.1)
			
				img = self.last_img
			
				corners = cv.FindChessboardCorners(img, (8,6))
				if corners[0]:
					cb_center = FindChessboardCenter(corners[1])
					self.data['cb_centers'].append(cb_center)
					cv.DrawChessboardCorners(img, (8,6), corners[1], corners[0])
					cb_pantilt = self.angle_from_cb_center(cb_center) - self.offset
					self.ground_truth_pub.publish(PanTilt(cb_pantilt[0], cb_pantilt[1], 0))
					recenter_ct = 0
				else:
					self.data['cb_centers'].append((np.nan, np.nan))
					recenter_ct += 1
				c = (int(orig_cb_center[0]), int(orig_cb_center[1]))
				cv.Circle(img, c, 2, (255,0,0), thickness=2)
				cv.SaveImage('calib_images/%sim_%s_%s.png' % (trial, pan, tilt), img)
			
				#cv.ShowImage('win', img)
				cv.WaitKey(10)
				action = (float(pan), float(tilt)) - self.last_pt
				self.last_pt = np.array((float(pan), float(tilt)))
				self.data['actions'].append(action)
				
				self.data['ground_truth'].append(cb_pantilt)
				print '%s: estimated pan/tilt (%d,%d), true pan/tilt (%d,%d)' % \
				tuple([trial, pan, tilt] + cb_pantilt.tolist())
		except KeyboardInterrupt: pass
		savemat('/opt/ros/packages/wu-ros-pkg/missouri/ptu_control/calib_data.mat', self.data)

	def image_cb(self, msg):
		self.last_img = self.bridge.imgmsg_to_cv(msg, 'passthrough')
		self.last_img_time = msg.header.stamp
		
	def angle_from_cb_center(self, pt):
		pan  = -atan2(self.C_X-pt[0], self.F_X)
		tilt = -atan2(self.C_Y-pt[1], self.F_Y)
		return np.degrees((pan, tilt))

if __name__ == '__main__':
	rospy.init_node('visual_calibration')
	
	pt_pub = rospy.Publisher('/pantilt', PanTilt)
	# pt = PanTilt(0,0,True)
	# pt_pub.publish(pt)
	# rospy.sleep(0.5)

	# pantiltReset(pt_pub)
	
	VisualCalibration()
