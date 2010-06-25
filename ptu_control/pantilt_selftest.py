#!/usr/bin/env python
import roslib; roslib.load_manifest('ptu_control')
import rospy
from logitech_pantilt.msg import PanTilt
from ptu_control.Calibration import pantiltReset

if __name__ == '__main__':
	rospy.init_node('pantilt_selftest')
	pt_pub = rospy.Publisher('/pantilt', PanTilt)
	pt = PanTilt(0,0,True)
	pt_pub.publish(pt)
	rospy.sleep(0.5)
	
	pantiltReset(pt_pub)
