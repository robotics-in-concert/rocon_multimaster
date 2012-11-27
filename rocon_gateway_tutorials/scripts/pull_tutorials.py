#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway_tutorials/LICENSE 
#
##############################################################################
# Imports
##############################################################################

import roslib; roslib.load_manifest('rocon_gateway_tutorials')
import rospy
import rocon_gateway
import rocon_gateway_tutorials
from gateway_msgs.msg import *
from gateway_msgs.srv import *
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
        self.req.cancel = cancel_flag
        self.req.remotes = []
        self.names, self.nodes = rocon_gateway_tutorials.createTutorialDictionaries(regex)

    def pull(self, type):
        rule = gateway_msgs.msg.Rule()
        rule.name = self.names[type]
        rule.type = type
        rule.node = self.nodes[type]
        self.req.remotes.append(RemoteRule(self.gateway, rule))
        rospy.loginfo("Pull : %s [%s,%s,%s,%s]."%(self.action_text, 
                                                  self.gateway, 
                                                  rule.type, 
                                                  rule.name, 
                                                  rule.node or 'None')) 
        resp = self.pull_service(self.req)
        if resp.result != 0:
            rospy.logerr("Pull : %s"%resp.error_message)
        self.req.remotes = []

##############################################################################
# Main
##############################################################################

"""
  Tests pulls, either for all tutorials (default) or one by one (via args).
  
  Usage:
    1 > roslaunch rocon_gateway_tutorials pirate.launch
    2a> roslaunch rocon_gateway_tutorials pirate_gateway_tutorials.launch
    3a> roslaunch rocon_gateway_tutorials pirate_gateway.launch
    2b> rosrun rocon_gateway_tutorials advertise_tutorials.py
    3b> rosrun rocon_gateway_tutorials pull_tutorials.py
    3c> rostopic echo /fibonacci/server/feedback
    3d> roslaunch rocon_gateway_tutorials fibonacci_client.launch
    3e> roslaunch rocon_gateway_tutorials fibonacci_server.launch
    2c> # wait for fibonnacci client to finish and close 
    3e> rosrun rocon_gateway_tutorials pull_tutorials.py --cancel
    2d> rosrun rocon_gateway_tutorials advertise_tutorials.py --cancel
    
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
        gateway = "pirate_gateway.*"
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
