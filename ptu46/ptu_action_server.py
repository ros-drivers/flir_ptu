#!/usr/bin/env python
import roslib; roslib.load_manifest('ptu46')
import rospy
import ptu_control.msg
import actionlib
from sensor_msgs.msg import JointState
import threading

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

		# setup the subscribers and publishers
		self.ptu_pub = rospy.publisher('cmd')
		self.as_goto = actionlib.SimpleActionServer('SetPTUState', \
		     ptu_control.msg.PtuGotoAction, execute_cb=self.cb_goto)
	
	def cb_goto(self, msg):
		#self.state_lock.acquire()
		pan, tilt, pan_vel, tilt_vel = msg.pan, msg.tilt, msg.pan_vel, msg.tilt_vel

		if not pan_vel:
			pan_vel = psmax
		if not tilt_vel:
			tilt_vel = tsmax
		
		msg_out = JointState()
		msg_out.header.stamp = rospy.Time.now()
		msg_out.name = ['head_pan_joint', 'head_tilt_joint']
		msg_out.position = [pan, tilt]
		msg_out.velocity = [pan_vel, tilt_vel]
		self.ptu_pub.publish(msg_out)

		result = ptu_control.msg.PtuGotoResult()
		# TODO this is WRONG and needs to be fixed (these values aren't necessarily right)
		result.state.position = [pan, tilt]
		self.as_goto.set_succeeded(result)

if __name__ == '__main__':
	rospy.init_node('ptu46_action_server')
	PTUControl()
	rospy.spin()
