#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway_tutorials/LICENSE 
#

import roslib; roslib.load_manifest('rocon_gateway_tutorials')
import rospy
import rocon_gateway
import rocon_gateway_tutorials
from gateway_msgs.msg import *
from gateway_msgs.srv import *
import argparse
import sys

"""
  Tests a single flip rule.
  
  Usage:
    1 > roslaunch rocon_gateway_tutorials pirate_hub.launch
    2a> roslaunch rocon_gateway_tutorials pirate_gateway_tutorials.launch
    3a> roslaunch rocon_gateway_tutorials pirate_gateway.launch
    2b> rosrun rocon_gateway_tutorials advertise_all.py
    3b> rosrun rocon_gateway_tutorials pull_all.py
"""

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Pull all connections (unpull if using --cancel')
    parser.add_argument('--cancel', action='store_true', help='cancel the pull')
    args = parser.parse_args()
    if args.cancel:
        action_text = "cancelling"
    else:
        action_text = "pull"

    rospy.init_node('pull_all')

    try:
        gateway = rocon_gateway_tutorials.findFirstRemoteGateway()
    except rocon_gateway.GatewayError as e:
        rospy.logerr("Pull All : %s, aborting."%(str(e)))
        sys.exit(1)
    
    flip_all = rospy.ServiceProxy('/gateway/pull_all',RemoteAll)
    req = RemoteAllRequest() 
    req.gateway = gateway
    req.cancel = args.cancel
    req.blacklist = []

    rospy.loginfo("Pull All : %s all [%s]."%(action_text,req.gateway)) 
    resp = flip_all(req)
    if resp.result != 0:
        rospy.logerr("Pull All : %s"%resp.error_message)

