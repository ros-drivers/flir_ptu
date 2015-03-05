#!/usr/bin/python
#This script uses socat to create a virtual com port that links to an IP device.
#The com port is created in the /tmp directory, so that sudo is not required.
#The script was originally designed for use with a FLIR D48E Pan-Tilt Unit,
#but should work fine with any IP device that accepts raw data via TCP/IP.

#Copyright: 2015, Clearpath Robotics
#Author: Jeff Schmidt 

import os, sys, time, argparse

parser = argparse.ArgumentParser()
parser.add_argument('--ip', default="192.168.1.17")
parser.add_argument('--com', default="/tmp/PTUcom")
parser.add_argument('--port', default="4000")
#args = parser.parse_args()
args, unknown = parser.parse_known_args()

# ping the desired IP to see if it's available
response = os.system("ping -c 1 " + (args.ip))

# if it responds, create a virtual com port.
if response == 0:
  print (args.ip), 'is up! Creating a virtual com port at', (args.com)
  os.system("socat pty,link=" + (args.com) + ",raw tcp:" + (args.ip) + ":" + (args.port) + "&")
#  time.sleep(10)
else:
  print (args.ip), 'is down! Cannot create virtual com port.'
