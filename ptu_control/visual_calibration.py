#!/usr/env/bin python
import roslib; roslib.load_manifest('ptu_control')
import rospy
import ptu_control.msg
import actionlib
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from scipy.io import savemat
import cv

class VisualCalibration(object):
	client = actionlib.SimpleActionClient('SetPTUState', ptu_control.msg.PtuGotoAction)
	bridge = CvBridge()
	
	last_img = None
	corner_data = {'corners':[]}
	
	
	def __init__(self):
		cv.NamedWindow('win')
		image_sub = rospy.Subscriber('image', Image, self.image_cb)
		self.client.wait_for_server()
		#goal = ptu_control.msg.PtuGotoGoal(pan=-70, tilt=-30)
		#self.client.send_goal(goal)

		while not self.last_img:
			rospy.sleep(0.1)

		for tilt in range(-20,21,5):
			for pan in range(-20,21,5):
				print pan, tilt
				goal = ptu_control.msg.PtuGotoGoal(pan=pan, tilt=tilt)
				self.client.send_goal(goal)
				self.client.wait_for_goal_to_finish()
				rospy.sleep(1)
				
				img = self.last_img
				corners = cv.FindChessboardCorners(img, (11,9))
				if corners[0]:
					#import pdb; pdb.set_trace()
					cv.SaveImage(\
				   '/project/robotics/users/lazewatskyd/wu-ros-pkg/missouri/ptu_control/calib_images/im_%s_%s.png'\
				   % (pan, tilt), img)
					cv.DrawChessboardCorners(img, (11,9), corners[1], corners[0])
					self.corner_data['corners'].append( (pan, tilt) + sum(corners[1], ()) )
				cv.ShowImage('win', img)
				cv.WaitKey(10)
				
		savemat('/project/robotics/users/lazewatskyd/wu-ros-pkg/missouri/ptu_control/calib_images/calib_data.mat',\
		        self.corner_data)

	def image_cb(self, msg):
		self.last_img = self.bridge.imgmsg_to_cv(msg, 'passthrough')

if __name__ == '__main__':
	rospy.init_node('visual_calibration')
	VisualCalibration()
