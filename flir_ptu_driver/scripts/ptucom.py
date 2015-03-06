#!/usr/bin/env python
#This script uses socat to create a virtual com port that links to an IP device.
#The com port is created in the /tmp directory, so that sudo is not required.
#The script was originally designed for use with a FLIR D48E Pan-Tilt Unit,
#but should work fine with any IP device that accepts raw data via TCP/IP.

#Copyright: 2015, Clearpath Robotics
#Author: Jeff Schmidt 

import os, sys, time, argparse
import rospy

rospy.init_node("flir_ptu_serial_bridge")

parser = argparse.ArgumentParser()
parser.add_argument('--ip', default="192.168.1.17")
parser.add_argument('--com', default="/tmp/PTUcom")
parser.add_argument('--port', default="4000")
args, unknown = parser.parse_known_args()

while not rospy.is_shutdown():
    try:
        # if it responds, create a virtual com port.
        rospy.loginfo('Creating a virtual com port at %s' % args.com)
        cmd = "socat pty,link=" + (args.com) + ",raw tcp:" + (args.ip) + ":" + (args.port)
        rospy.loginfo("Executing: %s" % cmd)
        os.system(cmd)
        rospy.logwarn("socat instance exited.")
    except Exception as e:
        rospy.logwarn("Exception thrown: %s" % e)

    time.sleep(1)
