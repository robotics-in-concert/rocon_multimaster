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
  Tests a single advertise rule.
  
  Usage:
    1 > roslaunch rocon_gateway_hub pirate.launch
    2a> roslaunch rocon_gateway pirate_chatter.launch
    3a> roslaunch rocon_gateway pirate.launch
    3b> rosrun rocon_gateway_tests advertise_chatter.py
    3c> rosservice call /gateway/gateway_info[]
    2b> rosservice call /gateway/remote_gateway_info []
"""

if __name__ == '__main__':

  parser = argparse.ArgumentParser(description='Flip /chatter to a remote gateway')
  parser.add_argument('--cancel', action='store_true', help='cancel the flip')
  args = parser.parse_args()

  rospy.init_node('advertise_chatter')

  remote_gateway_info = rospy.ServiceProxy('/gateway/remote_gateway_info',RemoteGatewayInfo)
  gateway_info = rospy.ServiceProxy('/gateway/gateway_info',GatewayInfo)
  advertise = rospy.ServiceProxy('/gateway/advertise',Advertise)
  
  req = RemoteGatewayInfoRequest()
  req.gateways = []
  resp = remote_gateway_info(req)
  if len(resp.gateways) == 0:
      rospy.logerr("Flip Test : no other gateways available to flip to, aborting.")
      sys.exit(1)
  else:
      gateway = resp.gateways[0].name
      
  # Form a request message
  req = AdvertiseRequest()
  req.rules = []
  public_rule = PublicRule()
  public_rule.connection.name = "/chatter"
  public_rule.connection.type = gateway_comms.msg.Connection.PUBLISHER
  #public_rule.connection.node = "/talker"
  req.rules.append(public_rule)
  req.cancel = args.cancel
  
  print ""
  print "== Request =="
  print ""
  print req
  print ""
  resp = advertise(req)
  print "== Response =="
  print ""
  print resp

  gateway_info_request = GatewayInfoRequest()
  gateway_info_response = gateway_info(gateway_info_request)
  print ""
  print "== Gateway Info =="
  print ""
  print gateway_info_response
  print ""
