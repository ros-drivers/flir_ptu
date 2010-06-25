#!/usr/bin/env python
import roslib; roslib.load_manifest('ptu_control')
import rospy
from sensor_msgs.msg import JointState
from logitech_pantilt.msg import PanTilt
from ptu_control.Calibration import pantiltReset
import actionlib
import ptu_control.msg
import threading

PAN_RANGE  = 70
TILT_RANGE = 30

class PTUControl(object):
	# setup some variables to keep track of the PTU's state (in degrees)
	pan      = 0
	tilt     = 0
	pan_vel  = 0
	tilt_vel = 0
	state_lock = threading.Lock()
	
	def __init__(self):
		# setup the subscribers and publishers
		self.joint_pub = rospy.Publisher('state', JointState)
		self.ptu_pub   = rospy.Publisher('/pantilt', PanTilt)
		self.as_goto   = actionlib.SimpleActionServer('SetPTUState', \
		     ptu_control.msg.PtuGotoAction, execute_cb=self.cb_goto)
		self.as_reset  = actionlib.SimpleActionServer('ResetPtu', \
			 ptu_control.msg.PtuResetAction, execute_cb=self.cb_reset)
		rospy.Subscriber('ground_truth_pantilt', PanTilt, self.ground_truth_cb)

	def cb_goto(self, msg):
		self.state_lock.acquire()
		pan, tilt, pan_vel, tilt_vel = msg.pan, msg.tilt, msg.pan_vel, msg.tilt_vel
		pan = min(pan, PAN_RANGE)
		pan = max(pan, -PAN_RANGE)
		tilt = min(tilt, TILT_RANGE)
		tilt = max(tilt, -TILT_RANGE)
		
		pan_cmd  = pan  - self.pan
		tilt_cmd = tilt - self.tilt
		
		self.pan  = pan
		self.tilt = tilt
		
		self.ptu_pub.publish(PanTilt(pan=pan_cmd,tilt=tilt_cmd,reset=False))
		
		result = ptu_control.msg.PtuGotoResult()
		result.state.position = [pan, tilt]
		self.state_lock.release()
		self.as_goto.set_succeeded(result)
		
	def cb_reset(self, msg):
		self.state_lock.acquire()
		pantiltReset(self.ptu_pub)
		result = ptu_control.msg.PtuResetResult()
		self.pan  = 0
		self.tilt = 0
		self.state_lock.release()
		self.as_reset.set_succeeded(result)
		
	def ground_truth_cb(self, msg):
		self.state_lock.acquire()
		self.pan = msg.pan
		self.tilt = msg.tilt
		self.state_lock.release()
		
if __name__ == '__main__':
	rospy.init_node('ptu_node')
	PTUControl()
	rospy.spin()
