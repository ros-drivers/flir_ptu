#!/usr/bin/env python
import roslib; roslib.load_manifest('missouri_telepresence')
import rospy
from logitech_pantilt.msg import PanTilt

PAN_STEP   = 2
TILT_STEP  = 2
SLEEP_TIME = 0.08

PAN_RANGE = 160
TILT_RANGE = 70

if __name__ == '__main__':
	rospy.init_node('pantilt_selftest')
	pt_pub = rospy.Publisher('/pantilt', PanTilt)
	pt = PanTilt(0,0,True)
	pt_pub.publish(pt)
	rospy.sleep(0.5)
		
	#move CCW
	for i in range(PAN_RANGE/PAN_STEP+1):
		pt = PanTilt(PAN_STEP, 0, False)
		pt_pub.publish(pt)
		rospy.sleep(SLEEP_TIME)
		
	#move CW
	for i in range(PAN_RANGE/PAN_STEP+1):
		pt = PanTilt(-PAN_STEP, 0, False)
		pt_pub.publish(pt)
		rospy.sleep(SLEEP_TIME)
		
	#move center
	for i in range(PAN_RANGE/2/PAN_STEP-1):
		pt = PanTilt(PAN_STEP, 0, False)
		pt_pub.publish(pt)
		rospy.sleep(SLEEP_TIME)
		
	#move up
	for i in range(TILT_RANGE/TILT_STEP+1):
		pt = PanTilt(0, TILT_STEP, False)
		pt_pub.publish(pt)
		rospy.sleep(SLEEP_TIME)
	
	#move down
	for i in range(TILT_RANGE/TILT_STEP+1):
		pt = PanTilt(0, -TILT_STEP, False)
		pt_pub.publish(pt)
		rospy.sleep(SLEEP_TIME)
		
	#move center
	for i in range(TILT_RANGE/2/TILT_STEP-1):
		pt = PanTilt(0, TILT_STEP, False)
		pt_pub.publish(pt)
		rospy.sleep(SLEEP_TIME)
	
