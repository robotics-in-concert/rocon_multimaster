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
    2a> roslaunch rocon_gateway pirate_chatter.launch
    3a> roslaunch rocon_gateway pirate.launch
    3b> rosrun rocon_gateway_tests flip_chatter.py
    2b> rostopic list
    3c> rosrun rocon_gateway_tests flip_chatter.py --cancel
    3d> rosrun rocon_gateway_tests flip_chatter.py --regex
    3c> rosrun rocon_gateway_tests flip_chatter.py --regex --cancel
"""

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Flip /chatter to a remote gateway')
    #parser.add_argument("gateway", help="gateway string identifier", type=str)
    parser.add_argument('--regex', action='store_true', help='test with a regex pattern')
    parser.add_argument('--cancel', action='store_true', help='cancel the flip')
    args = parser.parse_args()

    rospy.init_node('flip_chatter')

    remote_gateway_info = rospy.ServiceProxy('/gateway/remote_gateway_info',RemoteGatewayInfo)
    flip = rospy.ServiceProxy('/gateway/flip',Flip)
  
    req = RemoteGatewayInfoRequest()
    req.gateways = []
    resp = remote_gateway_info()
    if len(resp.gateways) == 0:
        rospy.logerr("Flip Test : no other gateways available to flip to, aborting.")
        sys.exit(1)
    else:
        gateway = resp.gateways[0].name
      
    # Form a request message
    req = FlipRequest() 
    if args.regex:
        req.flip_rule.connection.name = ".*ter"
    else:
        req.flip_rule.connection.name = "/chatter"
    req.flip_rule.connection.type = gateway_comms.msg.Connection.PUBLISHER
    req.flip_rule.gateway = gateway
    req.cancel = args.cancel
  
    print ""
    print "== Request =="
    print ""
    print req
    print ""
    
    resp = flip(req)
    print "== Response =="
    print ""
    print resp
    print ""

    req.flip_rule.connection.type = gateway_comms.msg.Connection.SUBSCRIBER
    resp = flip(req)
    print "== Response =="
    print ""
    print resp

