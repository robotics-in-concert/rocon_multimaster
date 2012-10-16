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
    2b> rosrun rocon_gateway_tests advertise_tutorials.py
    2c> rosservice call /gateway/gateway_info
    3b> rosservice call /gateway/remote_gateway_info []
    2d> rosrun rocon_gateway_tests advertise_tutorials.py --cancel
    2e> rosrun rocon_gateway_tests advertise_tutorials.py --regex
    2d> rosrun rocon_gateway_tests advertise_tutorials.py --regex --cancel
"""

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Advertise tutorial connections (/chatter, /add_two_ints to a remote gateway')
    parser.add_argument('--pubonly', action='store_true', help='flip /chatter publisher only')
    parser.add_argument('--subonly', action='store_true', help='flip /chatter subscriber only')
    parser.add_argument('--serviceonly', action='store_true', help='flip /chatter subscriber only')
    parser.add_argument('--regex', action='store_true', help='test with a regex pattern')
    parser.add_argument('--cancel', action='store_true', help='cancel the flip')
    args = parser.parse_args()
    advertise_all_connection_types = (not args.pubonly) and (not args.subonly) and (not args.serviceonly)
    if args.cancel:
        action_text = "cancelling"
    else:
        action_text = "advertising"

    rospy.init_node('advertise_tutorials')

    advertise = rospy.ServiceProxy('/gateway/advertise',Advertise)
    req = AdvertiseRequest() 
    req.cancel = args.cancel

    rule = Rule()
    if args.regex:
        rule.name = ".*ter"
        rule.node = "/t.*er"
    else:
        rule.name = "/chatter"

    if args.pubonly or advertise_all_connection_types:
        rule.type = gateway_comms.msg.ConnectionType.PUBLISHER
        rospy.loginfo("Advertise : %s [%s,%s,%s]."%(action_text, rule.type, rule.name, rule.node or 'None'))
        req.rules.append(rule) 
        resp = advertise(req)
        #if resp.result != 0:
        #    rospy.logerr("Advertise : %s"%resp.error_message)

    rule.node = ''
    req.rules = []
    
    if args.subonly or advertise_all_connection_types:
        rule.type = gateway_comms.msg.ConnectionType.SUBSCRIBER
        rospy.loginfo("Advertise : %s [%s,%s,%s]."%(action_text, rule.type, rule.name, rule.node or 'None')) 
        req.rules.append(rule) 
        resp = advertise(req)
        if resp.result != 0:
            rospy.logerr("Advertise : error occured (todo: no error message yet)")
            #rospy.logerr("Advertise : %s"%resp.error_message)

    if args.regex:
        rule.name = "/add_two_.*"
    else:
        rule.name = "/add_two_ints"

    req.rules = []

    if args.serviceonly or advertise_all_connection_types:
        rule.type = gateway_comms.msg.ConnectionType.SERVICE
        rospy.loginfo("Advertise : %s [%s,%s,%s]."%(action_text,rule.type, rule.name, rule.node or 'None')) 
        req.rules.append(rule) 
        resp = advertise(req)
        if resp.result != 0:
            rospy.logerr("Advertise : error occured (todo: no error message yet)")
            #rospy.logerr("Advertise : %s"%resp.error_message)
