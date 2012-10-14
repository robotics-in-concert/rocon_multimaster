#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway_tests/LICENSE 
#

import roslib; roslib.load_manifest('rocon_gateway_tests')
import rospy
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
    2b> rosrun rocon_gateway_tests flip_all.py
    2c> rosrun rocon_gateway_tests flip_all.py --cancel
"""

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Flip all connections (unflip if using --cancel')
    #parser.add_argument("gateway", help="gateway string identifier", type=str)
    parser.add_argument('--cancel', action='store_true', help='cancel the flip')
    args = parser.parse_args()
    if args.cancel:
        action_text = "cancelling"
    else:
        action_text = "flipping"

    rospy.init_node('flip_all')

    remote_gateway_info = rospy.ServiceProxy('/gateway/remote_gateway_info',RemoteGatewayInfo)
    flip_all = rospy.ServiceProxy('/gateway/flip_all',RemoteAll)
  
    req = RemoteGatewayInfoRequest()
    req.gateways = []
    resp = remote_gateway_info(req)
    if len(resp.gateways) == 0:
        rospy.logerr("Flip Test : no other gateways available to flip to, aborting.")
        sys.exit(1)
    else:
        gateway = resp.gateways[0].name
      
    # Form a request message
    req = RemoteAllRequest() 
    req.gateway = gateway
    req.cancel = args.cancel
    req.blacklist = []

    rospy.loginfo("Flip All : %s all [%s]."%(action_text,req.gateway)) 
    resp = flip_all(req)
    if resp.result != 0:
        rospy.logerr("Flip All : %s"%resp.error_message)

