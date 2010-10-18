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

PAN_RANGE  = 70
TILT_RANGE = 30

class PTUControl(object):
	# setup some variables to keep track of the PTU's state (in degrees)
	pan      = 0
	tilt     = 0
	pan_vel  = 0
	tilt_vel = 0
	state_lock = threading.Lock()
	
	kf = ptu_control.ptu_tracker.PanTiltKF()
	
	def __init__(self, reset=True):
		# setup the subscribers and publishers
		self.joint_pub = rospy.Publisher('state', JointState)
		self.ptu_pub   = rospy.Publisher('/pantilt', PanTilt)
		self.as_goto   = actionlib.SimpleActionServer('SetPTUState', \
		     ptu_control.msg.PtuGotoAction, execute_cb=self.cb_goto)
		self.as_reset  = actionlib.SimpleActionServer('ResetPtu', \
			 ptu_control.msg.PtuResetAction, execute_cb=self.cb_reset)
		rospy.Subscriber('ground_truth_pantilt', PanTilt, self.ground_truth_cb)
		
		br = tf.TransformBroadcaster()
		threading.Thread(target=self.send_transform).start()
    		
	if reset:
		rospy.sleep(1.0)
		pantiltReset(self.ptu_pub)

	def cb_goto(self, msg):
		self.state_lock.acquire()
		pan, tilt, pan_vel, tilt_vel = msg.pan, msg.tilt, msg.pan_vel, msg.tilt_vel
		pan = min(pan, PAN_RANGE)
		pan = max(pan, -PAN_RANGE)
		tilt = min(tilt, TILT_RANGE)
		tilt = max(tilt, -TILT_RANGE)
				
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
		
		m = geometry_msgs.msg.TransformStamped()
		m.header.frame_id = 'ptu'

		m.transform.rotation.x = quat[0]
		m.transform.rotation.y = quat[1]
		m.transform.rotation.z = quat[2]
		m.transform.rotation.w = quat[3]
		
		
		
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
		while not rospy.is_shutdown():
			self.br.sendTransform(
				(0,0,0)
				quat = tf.transformations.quaternion_from_euler(0, self.tilt, self.pan),
				rospy.Time.now(),
				'ptu',
				'odom'
			)
		
if __name__ == '__main__':
	rospy.init_node('ptu_node')
	PTUControl()
	rospy.spin()
