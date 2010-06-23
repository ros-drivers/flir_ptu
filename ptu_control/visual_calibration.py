#!/usr/env/bin python
import roslib; roslib.load_manifest('ptu_control')
import rospy
import ptu_control.msg
import actionlib
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from scipy.io import savemat
from random import randint
import cv
import numpy as np

from logitech_pantilt.msg import PanTilt
from ptu_control.Calibration import pantiltReset



class VisualCalibration(object):
	client = actionlib.SimpleActionClient('SetPTUState', ptu_control.msg.PtuGotoAction)
	bridge = CvBridge()
	
	last_img = None
	data = {'cb_centers':[], 'actions':[]}
	last_pt = np.array([0,0])
	
	
	def __init__(self):
		self.last_img_time = rospy.Time.now()
		cv.NamedWindow('win')
		image_sub = rospy.Subscriber('image', Image, self.image_cb)
		self.client.wait_for_server()

		# wait for images to start coming in
		while not self.last_img:
			rospy.sleep(0.1)
		
		PAN_RANGE  = 10
		TILT_RANGE = 10
		
		# while not rospy.is_shutdown():
		for trial in range(1000):
			if rospy.is_shutdown(): break
			pan = randint(-PAN_RANGE,PAN_RANGE)
			tilt = randint(-TILT_RANGE,TILT_RANGE)
			print 'trial: %s \t -> (%s, %s)' % (trial, pan, tilt)
			
			goal = ptu_control.msg.PtuGotoGoal(pan=pan, tilt=tilt)
			self.client.send_goal(goal)
			self.client.wait_for_goal_to_finish()
			finish_time = rospy.Time.now()
			rospy.sleep(1.0)
			
			while self.last_img_time < finish_time:
				rospy.sleep(0.1)
			
			img = self.last_img
			
			
			corners = cv.FindChessboardCorners(img, (8,6))
			if corners[0]:
				corners_np = np.array(corners[1])
				cb_center = np.average(corners_np, axis=0)
				self.data['cb_centers'].append(cb_center)
				cv.DrawChessboardCorners(img, (8,6), corners[1], corners[0])
			else:
				self.data['cb_centers'].append((np.nan, np.nan))

			cv.SaveImage('calib_images/%sim_%s_%s.png' % (trial, pan, tilt), img)
			
			cv.ShowImage('win', img)
			cv.WaitKey(10)
			action = (float(pan), float(tilt)) - self.last_pt
			self.last_pt = np.array((float(pan), float(tilt)))
			self.data['actions'].append(action)
		savemat('calib_data.mat', self.data)

	def image_cb(self, msg):
		self.last_img = self.bridge.imgmsg_to_cv(msg, 'passthrough')
		self.last_img_time = msg.header.stamp

if __name__ == '__main__':
	rospy.init_node('visual_calibration')
	
	pt_pub = rospy.Publisher('/pantilt', PanTilt)
	pt = PanTilt(0,0,True)
	pt_pub.publish(pt)
	rospy.sleep(0.5)

	pantiltReset(pt_pub)
	
	VisualCalibration()
