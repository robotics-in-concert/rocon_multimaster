#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway_tests/LICENSE 
#
##############################################################################
# Imports
##############################################################################

import roslib; roslib.load_manifest('rocon_gateway_tests')
import rospy
import rocon_gateway
import rocon_gateway_tests
from gateway_comms.msg import *
from gateway_comms.srv import *
import argparse
import sys

##############################################################################
# Utilities
##############################################################################

class Context(object):
    def __init__(self, gateway, cancel_flag, regex):
        self.gateway = gateway
        if cancel_flag:
            self.action_text = "cancelling"
        else:
            self.action_text = "flipping"
        self.flip_service = rospy.ServiceProxy('/gateway/flip',Remote)
        self.req = RemoteRequest() 
        self.req.gateway = gateway
        self.req.cancel = cancel_flag
        self.req.rules = []
        self.names, self.nodes = rocon_gateway_tests.createTutorialDictionaries(regex)

    def flip(self, type):
        rule = gateway_comms.msg.Rule()
        rule.name = self.names[type]
        rule.type = type
        rule.node = self.nodes[type]
        self.req.rules.append(rule)
        rospy.loginfo("Flip : %s [%s,%s,%s,%s]."%(self.action_text, 
                                                  self.req.gateway, 
                                                  rule.type, 
                                                  rule.name, 
                                                  rule.node or 'None')) 
        resp = self.flip_service(self.req)
        if resp.result != 0:
            rospy.logerr("Flip : %s"%resp.error_message)
        self.req.rules = []

##############################################################################
# Main
##############################################################################

"""
  Tests flips, either for all tutorials (default) or one by one (via args).
  
  Usage:
    1 > roslaunch rocon_gateway_hub pirate.launch

    2a> roslaunch rocon_gateway pirate_tutorials.launch
    3a> roslaunch rocon_gateway pirate.launch

    2b> rosrun rocon_gateway_tests flip_tutorials.py
    3b> rostopic list

    2c> rosrun rocon_gateway_tests flip_tutorials.py --cancel
    2d> rosrun rocon_gateway_tests flip_tutorials.py --regex
    2e> rosrun rocon_gateway_tests flip_tutorials.py --regex --cancel
"""

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Flip roscpp tutorial connections (/chatter, /add_two_ints to a remote gateway')
    parser.add_argument('--pubonly', action='store_true', help='flip /chatter publisher only')
    parser.add_argument('--subonly', action='store_true', help='flip /chatter subscriber only')
    parser.add_argument('--serviceonly', action='store_true', help='flip add_two_ints service only')
    parser.add_argument('--actionclientonly', action='store_true', help='flip /fibonacci action client only')
    parser.add_argument('--actionserveronly', action='store_true', help='flip /averaging action server only')
    parser.add_argument('--regex', action='store_true', help='test with a regex pattern')
    parser.add_argument('--cancel', action='store_true', help='cancel the flip')
    args = parser.parse_args()
    flip_all_connection_types = (not args.pubonly) and (not args.subonly) and (not args.serviceonly) and (not args.actionclientonly) and (not args.actionserveronly)
    if args.cancel:
        action_text = "cancelling"
    else:
        action_text = "flipping"

    rospy.init_node('flip_tutorials')

    try:
        gateway = rocon_gateway_tests.findFirstRemoteGateway()
    except rocon_gateway.GatewayError as e:
        rospy.logerr("Flip Test : %s, aborting."%(str(e)))
        sys.exit(1)

    context = Context(gateway, args.cancel, args.regex)

    if args.pubonly or flip_all_connection_types:
        context.flip(ConnectionType.PUBLISHER)
    
    if args.subonly or flip_all_connection_types:
        context.flip(ConnectionType.SUBSCRIBER)

    if args.serviceonly or flip_all_connection_types:
        context.flip(ConnectionType.SERVICE)

    if args.actionclientonly or flip_all_connection_types:
        context.flip(ConnectionType.ACTION_CLIENT)

    if args.actionserveronly or flip_all_connection_types:
        context.flip(ConnectionType.ACTION_SERVER)
