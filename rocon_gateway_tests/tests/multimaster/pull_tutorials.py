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
            self.action_text = "pulling"
        self.pull_service = rospy.ServiceProxy('/gateway/pull',Remote)
        self.req = RemoteRequest() 
        self.req.remote.gateway = gateway
        self.req.cancel = cancel_flag
        self.names, self.nodes = rocon_gateway_tests.createTutorialDictionaries(regex)

    def pull(self, type):
        self.req.remote.rule.name = self.names[type]
        self.req.remote.rule.type = type
        self.req.remote.rule.node = self.nodes[type]
        rospy.loginfo("Pull : %s [%s,%s,%s,%s]."%(self.action_text, 
                                                  self.req.remote.gateway, 
                                                  self.req.remote.rule.type, 
                                                  self.req.remote.rule.name, 
                                                  self.req.remote.rule.node or 'None')) 
        resp = self.pull_service(self.req)
        if resp.result != 0:
            rospy.logerr("Pull : %s"%resp.error_message)

##############################################################################
# Main
##############################################################################

"""
  Tests pulls, either for all tutorials (default) or one by one (via args).
  
  Usage:
    1 > roslaunch rocon_gateway_hub pirate.launch
    2a> roslaunch rocon_gateway pirate_tutorials.launch
    2b> rosrun rocon_gateway_tests advertise_tutorials.py
    3a> roslaunch rocon_gateway pirate.launch
    3b> rosrun rocon_gateway_tests pull_tutorials.py
    3c> rostopic echo /fibonacci/server/feedback
    3d> roslaunch rocon_gateway_tests fibonacci_client.launch
    3e> rosrun rocon_gateway_tests pull_tutorials.py --cancel
    2c> rosrun rocon_gateway_tests advertise_tutorials.py --cancel
    
  Variations in the options (singly, or by regex patterns).
"""

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Pull roscpp tutorial connections (/chatter, /add_two_ints to a remote gateway')
    parser.add_argument('--pubonly', action='store_true', help='pull /chatter publisher only')
    parser.add_argument('--subonly', action='store_true', help='pull /chatter subscriber only')
    parser.add_argument('--serviceonly', action='store_true', help='pull add_two_ints service only')
    parser.add_argument('--actionclientonly', action='store_true', help='pull /fibonacci action client only')
    parser.add_argument('--actionserveronly', action='store_true', help='pull /averaging action server only')
    parser.add_argument('--regex', action='store_true', help='test with a regex pattern')
    parser.add_argument('--cancel', action='store_true', help='cancel the pull')
    args = parser.parse_args()
    pull_all_connection_types = (not args.pubonly) and (not args.subonly) and (not args.serviceonly) and (not args.actionclientonly) and (not args.actionserveronly)
    if args.cancel:
        action_text = "cancelling"
    else:
        action_text = "pulling"

    rospy.init_node('pull_tutorials')

    try:
        gateway = rocon_gateway_tests.findFirstRemoteGateway()
    except rocon_gateway.GatewayError as e:
        rospy.logerr("Pull Test : %s, aborting."%(str(e)))
        sys.exit(1)

    context = Context(gateway, args.cancel, args.regex)

    if args.pubonly or pull_all_connection_types:
        context.pull(ConnectionType.PUBLISHER)
    
    if args.subonly or pull_all_connection_types:
        context.pull(ConnectionType.SUBSCRIBER)

    if args.serviceonly or pull_all_connection_types:
        context.pull(ConnectionType.SERVICE)

    if args.actionclientonly or pull_all_connection_types:
        context.pull(ConnectionType.ACTION_CLIENT)

    if args.actionserveronly or pull_all_connection_types:
        context.pull(ConnectionType.ACTION_SERVER)
