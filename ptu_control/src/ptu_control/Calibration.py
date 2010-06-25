#!/usr/bin/env python
import roslib; roslib.load_manifest('ptu_control')
from logitech_pantilt.msg import PanTilt
import rospy
import cv
import os

TEMPLATE = cv.LoadImage(os.popen('rospack find ptu_control').read().strip() + '/template.jpg')
w, h = cv.GetSize(TEMPLATE)

SLEEP_TIME = 1.5

PAN_RANGE = 160
TILT_RANGE = 70

def getCenterPixelFromCalib(img):
	W, H = cv.GetSize(img)
	result = cv.CreateImage((W-w+1, H-h+1), cv.IPL_DEPTH_32F, 1)
	cv.MatchTemplate(img, TEMPLATE, result, cv.CV_TM_SQDIFF_NORMED)
	locs = cv.MinMaxLoc(result)[2]
	locs[0] += TEMPLATE.width/2
	locs[1] += TEMPLATE.height/2
	return locs
	
def pantiltReset(publisher):
	publisher.publish(PanTilt(PAN_RANGE, TILT_RANGE, False))
	rospy.sleep(SLEEP_TIME)

	publisher.publish(PanTilt(-PAN_RANGE/2, -TILT_RANGE/2, False))
	rospy.sleep(SLEEP_TIME)