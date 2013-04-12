#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway_tutorials/LICENSE 
#
##############################################################################
# Imports
##############################################################################

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
    def __init__(self, cancel_flag, regex):
        if cancel_flag:
            self.action_text = "cancelling"
        else:
            self.action_text = "advertising"
        self.advertise_service = rospy.ServiceProxy('/gateway/advertise',Advertise)
        self.req = AdvertiseRequest() 
        self.req.cancel = cancel_flag
        self.rule = Rule()
        self.names, self.nodes = rocon_gateway_tutorials.createTutorialDictionaries(regex)
        
    def advertise(self, type):
        self.req.rules = []
        self.rule.name = self.names[type]
        self.rule.type = type
        self.rule.node = self.nodes[type]
        rospy.loginfo("Advertise : %s [%s,%s,%s]."%(self.action_text,self.rule.type, self.rule.name, self.rule.node or 'None')) 
        self.req.rules.append(self.rule) 
        resp = self.advertise_service(self.req)
        if resp.result != 0:
            rospy.logerr("Advertise : %s"%resp.error_message)

##############################################################################
# Main
##############################################################################
    
"""
  Tests advertisements, either for all tutorials (default) or one by one (via args).
  
  Usage:
    1 > roslaunch rocon_gateway_tutorials pirate_hub.launch
    2a> roslaunch rocon_gateway_tutorials pirate_gateway_tutorials.launch
    3a> roslaunch rocon_gateway_tutorials pirate_gateway.launch
    2b> rosrun rocon_gateway_tutorials advertise_tutorials.py
    2c> rosservice call /gateway/gateway_info
    3b> rosservice call /gateway/remote_gateway_info []
    2d> rosrun rocon_gateway_tutorials advertise_tutorials.py --cancel
    2e> rosrun rocon_gateway_tutorials advertise_tutorials.py --regex
    2f> rosrun rocon_gateway_tutorials advertise_tutorials.py --regex --cancel
"""

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Advertise tutorial connections (/chatter, /add_two_ints to a remote gateway')
    parser.add_argument('--pubonly', action='store_true', help='advertise /chatter publisher only')
    parser.add_argument('--subonly', action='store_true', help='advertise /chatter subscriber only')
    parser.add_argument('--serviceonly', action='store_true', help='advertise /add_two_ints service only')
    parser.add_argument('--actionclientonly', action='store_true', help='advertise /fibonacci action client only')
    parser.add_argument('--actionserveronly', action='store_true', help='advertise /averaging action server only')
    parser.add_argument('--regex', action='store_true', help='test with a regex pattern')
    parser.add_argument('--cancel', action='store_true', help='cancel the advertisement')
    args = parser.parse_args()

    advertise_all_connection_types = (not args.pubonly) and (not args.subonly) and (not args.serviceonly) and (not args.actionclientonly) and (not args.actionserveronly)

    rospy.init_node('advertise_tutorials')

    context = Context(args.cancel, args.regex)

    if args.pubonly or advertise_all_connection_types:
        context.advertise(ConnectionType.PUBLISHER)

    if args.subonly or advertise_all_connection_types:
        context.advertise(ConnectionType.SUBSCRIBER)

    if args.serviceonly or advertise_all_connection_types:
        context.advertise(ConnectionType.SERVICE)

    if args.actionclientonly or advertise_all_connection_types:
        context.advertise(ConnectionType.ACTION_CLIENT)

    if args.actionserveronly or advertise_all_connection_types:
        context.advertise(ConnectionType.ACTION_SERVER)

