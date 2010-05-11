#!/usr/env/bin python
import roslib; roslib.load_manifest('ptu_control')
import rospy
from sensor_msgs.msg import JointState
from logitech_pantilt.msg import PanTilt
import actionlib
import ptu_control.msg

class PTUControl(object):
	# setup some variables to keep track of the PTU's state
	pan      = 0
	tilt     = 0
	pan_vel  = 0
	tilt_vel = 0
	
	# setup the subscribers and publishers
	joint_sub = rospy.Subscriber('cmd', JointState, self.set_goal)
	joint_pub = rospy.Publisher('state', JointState)
	ptu_pub   = rospy.Publisher('/pantilt', PanTilt)
	
	def __init__(self):
		self.as_goto = actionlib.SimpleActionServer('SetPTUState', \
		     ptu_control.msg.PtuGotoAction, execute_cb=self.cb_goto)
		self.as_setvel = actionlib.SimpleActionServer('SetPTUState', \
			 ptu_control.msg.PtuGotoAction, execute_cb=self.cb_setvel)
	def cb_goto(self, msg):
		pan, tilt, pan_vel, tilt_vel = msg.pan, msg.tilt, msg.pan_vel, msg.tilt_vel
		if pan_vel == 0 and tilt_vel == 0: # no vel defined, just go there
			pass
		else: # go at the specified velocity
			pass
	
	def cb_setvel(self, msg):
		pan_vel = msg.pan_vel
		tilt_vel = msg.tilt_vel
		

		
if __name__ == '__main__':
	rospy.init_node('ptu_node')
	PTUControl()