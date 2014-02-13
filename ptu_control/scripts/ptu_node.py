#!/usr/bin/env python
import roslib; roslib.load_manifest('ptu_control')
import rospy
from sensor_msgs.msg import JointState
from logitech_pantilt.msg import PanTilt
from ptu_control.Calibration import pantiltReset
import actionlib
import ptu_control.ptu_tracker
import ptu_control.msg
import threading

import tf
import geometry_msgs.msg


class PTUControl(object):
	# setup some variables to keep track of the PTU's state (in degrees)
	pan      = 0
	tilt     = 0
	pan_vel  = 0
	tilt_vel = 0
	state_lock = threading.Lock()
	
	kf = ptu_control.ptu_tracker.PanTiltKF()
	
	def __init__(self, reset=True):
		self.PAN_RANGE		= rospy.get_param('pan_range', 70)
		self.TILT_RANGE		= rospy.get_param('tilt_range', 30)
		self.PARENT_FRAME	= rospy.get_param('parent_frame', 'odom')
		self.PTU_FRAME		= rospy.get_param('parent_frame', 'ptu')
		
		
		# setup the subscribers and publishers
		self.joint_pub = rospy.Publisher('state', JointState)
		self.ptu_pub   = rospy.Publisher('/pantilt', PanTilt)
		self.as_goto   = actionlib.SimpleActionServer('SetPTUState', \
		     ptu_control.msg.PtuGotoAction, execute_cb=self.cb_goto)
		self.as_reset  = actionlib.SimpleActionServer('ResetPtu', \
			 ptu_control.msg.PtuResetAction, execute_cb=self.cb_reset)
		rospy.Subscriber('ground_truth_pantilt', PanTilt, self.ground_truth_cb)
		
		self.br = tf.TransformBroadcaster()
		threading.Thread(target=self.send_transform).start()
    		
		if reset:
			rospy.sleep(1.0)
			pantiltReset(self.ptu_pub)

	def cb_goto(self, msg):
		self.state_lock.acquire()
		pan, tilt, pan_vel, tilt_vel = msg.pan, msg.tilt, msg.pan_vel, msg.tilt_vel
		pan = min(pan, self.PAN_RANGE)
		pan = max(pan, -self.PAN_RANGE)
		tilt = min(tilt, self.TILT_RANGE)
		tilt = max(tilt, -self.TILT_RANGE)
				
		pan_cmd  = pan  - self.pan
		tilt_cmd = tilt - self.tilt
		
		self.pan, self.tilt = self.kf.control((pan_cmd, tilt_cmd))
				
		# self.pan  = pan
		# self.tilt = tilt
		
		if pan_cmd == 0 and tilt_cmd == 0:
			pass
		else:
			self.ptu_pub.publish(PanTilt(pan=pan_cmd,tilt=tilt_cmd,reset=False))
		
		result = ptu_control.msg.PtuGotoResult()
		result.state.position = [self.pan, self.tilt]
		self.state_lock.release()
		self.as_goto.set_succeeded(result)
			
		#TODO figure out when we're actually finished
		
	def cb_reset(self, msg):
		self.kf.__init__()
		
		self.state_lock.acquire()
		pantiltReset(self.ptu_pub)
		result = ptu_control.msg.PtuResetResult()
		self.pan  = 0
		self.tilt = 0
		self.state_lock.release()
		self.as_reset.set_succeeded(result)
		
	def ground_truth_cb(self, msg):
		self.state_lock.acquire()
		# self.pan = msg.pan
		# self.tilt = msg.tilt
		self.pan, self.tilt = self.kf.measurement((msg.pan, msg.tilt))
		self.state_lock.release()
		
		
	def send_transform(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.br.sendTransform(
				(0,0,0),
				tf.transformations.quaternion_from_euler(0, self.tilt, self.pan),
				rospy.Time.now(),
				self.PTU_FRAME,
				self.PARENT_FRAME
			)
			rate.sleep()
		
if __name__ == '__main__':
	rospy.init_node('ptu_node')
	PTUControl()
	rospy.spin()
