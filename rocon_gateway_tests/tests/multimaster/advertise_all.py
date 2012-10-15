#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway_tests/LICENSE 
#

import roslib; roslib.load_manifest('rocon_gateway_tests')
import rospy
import rocon_gateway
import rocon_gateway_tests
from gateway_comms.msg import *
from gateway_comms.srv import *
import argparse
import sys

"""
  Tests a single flip rule.
  
  Usage:
    1 > roslaunch rocon_gateway_hub pirate.launch
    2a> roslaunch rocon_gateway pirate_tutorials.launch
    3a> roslaunch rocon_gateway pirate.launch
    2b> rosrun rocon_gateway_tests advertise_all.py
    2c> rosrun rocon_gateway_tests advertise_all.py --cancel
"""

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Advertise all connections (unadvertise if using --cancel')
    parser.add_argument('--cancel', action='store_true', help='cancel the advertisements')
    args = parser.parse_args()
    if args.cancel:
        action_text = "cancelling"
    else:
        action_text = "advertising"

    rospy.init_node('advertise_all')
    
    advertise_all = rospy.ServiceProxy('/gateway/advertise_all',AdvertiseAll)
    req = AdvertiseAllRequest() 
    req.cancel = args.cancel
    req.blacklist = []

    rospy.loginfo("Advertise All : %s all."%action_text) 
    resp = advertise_all(req)
    if resp.result != 0:
        rospy.logerr("Advertise All : %s"%resp.error_message)

