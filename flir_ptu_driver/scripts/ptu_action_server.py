#!/usr/bin/env python
import roslib; roslib.load_manifest('ptu46')
import rospy
import ptu_control.msg
import actionlib
from sensor_msgs.msg import JointState
import threading
import numpy as np

class PTUControl(object):
	pan      = 0
	tilt     = 0
	pan_vel  = 0
	tilt_vel = 0
	state_lock = threading.Lock()

	def __init__(self):
		# setup some parameters
		self.tsmin = rospy.get_param('/ptu/min_tilt_speed', 4.0)
		self.tsmax = rospy.get_param('/ptu/max_tilt_speed', 140.0)
		self.psmin = rospy.get_param('/ptu/min_pan_speed' , 4.0)
		self.psmax = rospy.get_param('/ptu/max_pan_speed' , 140.0)
		self.pstep = rospy.get_param('/ptu/pan_step', 0.00089759763795882463)
		self.tstep = rospy.get_param('/ptu/tilt_step', 0.00089759763795882463)

		# setup the subscribers and publishers
		rospy.Subscriber('state', JointState, self.cb_ptu_state)
		self.ptu_pub = rospy.Publisher('cmd', JointState)
		self.as_goto = actionlib.SimpleActionServer('SetPTUState', \
		     ptu_control.msg.PtuGotoAction, execute_cb=self.cb_goto)
		self.as_reset  = actionlib.SimpleActionServer('ResetPtu', \
			 ptu_control.msg.PtuResetAction, execute_cb=self.cb_reset)
	
	def cb_goto(self, msg):
		pan, tilt, pan_vel, tilt_vel = np.radians((msg.pan, msg.tilt, msg.pan_vel, msg.tilt_vel))

		if not pan_vel:
			pan_vel = self.psmax
		if not tilt_vel:
			tilt_vel = self.tsmax

		self._goto(pan, tilt, pan_vel, tilt_vel)

		result = ptu_control.msg.PtuGotoResult()
		result.state.position = self._get_state()
		self.as_goto.set_succeeded(result)
		
	def cb_reset(self, msg):
		self._goto(0,0, self.psmax, self.tsmax)
		result = ptu_control.msg.PtuResetResult()
		self.as_reset.set_succeeded(result)

	def _goto(self, pan, tilt, pan_vel, tilt_vel):
		rospy.loginfo('going to (%s, %s)' % (pan, tilt))
		msg_out = JointState()
		msg_out.header.stamp = rospy.Time.now()
		msg_out.name = ['head_pan_joint', 'head_tilt_joint']
		msg_out.position = [pan, tilt]
		msg_out.velocity = [pan_vel, tilt_vel]
		self.ptu_pub.publish(msg_out)
		# wait for it to get there
		wait_rate = rospy.Rate(10)
		while not self._at_goal((pan, tilt)) and not rospy.is_shutdown():
			wait_rate.sleep()

	def _at_goal(self, goal):
		return all(np.abs(np.array(goal) - (self.pan, self.tilt)) <= np.degrees((self.pstep, self.tstep)))

	def cb_ptu_state(self, msg):
		self.state_lock.acquire()
		self.pan, self.tilt = msg.position
		self.state_lock.release()

	def _get_state(self):
		self.state_lock.acquire()
		pt = np.degrees((self.pan, self.tilt))
		self.state_lock.release()
		return pt

if __name__ == '__main__':
	rospy.init_node('ptu46_action_server')
	PTUControl()
	rospy.spin()
