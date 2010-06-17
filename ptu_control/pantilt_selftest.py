#!/usr/bin/env python
import roslib; roslib.load_manifest('missouri_telepresence')
import rospy
from logitech_pantilt.msg import PanTilt

PAN_STEP   = 2
TILT_STEP  = 2
SLEEP_TIME = 2.0

PAN_RANGE = 160
TILT_RANGE = 70

if __name__ == '__main__':
	rospy.init_node('pantilt_selftest')
	pt_pub = rospy.Publisher('/pantilt', PanTilt)
	pt = PanTilt(0,0,True)
	pt_pub.publish(pt)
	rospy.sleep(0.5)
	
	pt_pub.publish(PanTilt(PAN_RANGE, 0, False))
	rospy.sleep(SLEEP_TIME)
	
	pt_pub.publish(PanTilt(-PAN_RANGE/2, 0, False))
	rospy.sleep(SLEEP_TIME)
	
	pt_pub.publish(PanTilt(0, TILT_RANGE, False))
	rospy.sleep(SLEEP_TIME)	

	pt_pub.publish(PanTilt(0, -TILT_RANGE/2, False))
	rospy.sleep(SLEEP_TIME)
