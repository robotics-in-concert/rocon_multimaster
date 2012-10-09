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
    3b> rosrun rocon_gateway_tests flip_tutorials.py
    2b> rostopic list
    3c> rosrun rocon_gateway_tests flip_tutorials.py --cancel
    3d> rosrun rocon_gateway_tests flip_tutorials.py --regex
    3c> rosrun rocon_gateway_tests flip_tutorials.py --regex --cancel
"""

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Flip roscpp tutorial connections (/chatter, /add_two_ints to a remote gateway')
    #parser.add_argument("gateway", help="gateway string identifier", type=str)
    parser.add_argument('--pubonly', action='store_true', help='flip /chatter publisher only')
    parser.add_argument('--subonly', action='store_true', help='flip /chatter subscriber only')
    parser.add_argument('--serviceonly', action='store_true', help='flip /chatter subscriber only')
    parser.add_argument('--regex', action='store_true', help='test with a regex pattern')
    parser.add_argument('--cancel', action='store_true', help='cancel the flip')
    args = parser.parse_args()
    flip_all_connection_types = (not args.pubonly) and (not args.subonly) and (not args.serviceonly)
    if args.cancel:
        action_text = "cancelling"
    else:
        action_text = "flipping"

    rospy.init_node('flip_tutorials')

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
    req.flip_rule.gateway = gateway
    req.cancel = args.cancel

    if args.regex:
        req.flip_rule.connection.name = ".*ter"
    else:
        req.flip_rule.connection.name = "/chatter"

    if args.pubonly or flip_all_connection_types:
        req.flip_rule.connection.type = gateway_comms.msg.Connection.PUBLISHER
        rospy.loginfo("Flip : %s [%s,%s,%s,%s]."%(action_text,req.flip_rule.gateway, req.flip_rule.connection.type, req.flip_rule.connection.name, req.flip_rule.connection.node or 'None')) 
        resp = flip(req)
        if resp.result != 0:
            rospy.logerr("Flip : %s"%resp.error_message)

    if args.subonly or flip_all_connection_types:
        req.flip_rule.connection.type = gateway_comms.msg.Connection.SUBSCRIBER
        rospy.loginfo("Flip : %s [%s,%s,%s,%s]."%(action_text,req.flip_rule.gateway, req.flip_rule.connection.type, req.flip_rule.connection.name, req.flip_rule.connection.node or 'None')) 
        resp = flip(req)
        if resp.result != 0:
            rospy.logerr("Flip : %s"%resp.error_message)

    if args.regex:
        req.flip_rule.connection.name = "/add_two_.*"
    else:
        req.flip_rule.connection.name = "/add_two_ints"

    if args.serviceonly or flip_all_connection_types:
        req.flip_rule.connection.type = gateway_comms.msg.Connection.SERVICE
        rospy.loginfo("Flip : %s [%s,%s,%s,%s]."%(action_text,req.flip_rule.gateway, req.flip_rule.connection.type, req.flip_rule.connection.name, req.flip_rule.connection.node or 'None')) 
        resp = flip(req)
        if resp.result != 0:
            rospy.logerr("Flip : %s"%resp.error_message)
