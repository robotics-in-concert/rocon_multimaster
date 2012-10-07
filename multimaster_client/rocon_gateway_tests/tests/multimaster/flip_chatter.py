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

"""
  Tests a single flip rule.
  
  Usage:
    1 > roslaunch rocon_gateway_hub pirate.launch
    2a> roslaunch rocon_gateway pirate_chatter.launch
    3a> roslaunch rocon_gateway pirate.launch
    3b> rosrun rocon_gateway_tests flip_chatter gateway1
    2b> rostopic list
"""

if __name__ == '__main__':

  parser = argparse.ArgumentParser(description='Flip /chatter to a remote gateway')
  parser.add_argument("gateway", help="gateway string identifier", type=str)
#  parser.add_argument('-c','--clients',metavar='<Client name>',type=str,nargs='+',help='Client\'s unique name on hub')
#  parser.add_argument('-m','--message',metavar='<Topic triple>',type=str,nargs='+',help='<Topic triple>="<topic name>,<topic type>,<node uri>"')
  args = parser.parse_args()

  rospy.init_node('flip_publisher')

  flip = rospy.ServiceProxy('/gateway/flip',Flip)
  
  # Form a request message
  req = FlipRequest() 
  req.flip_rule.connection.name = "/chatter"
  req.flip_rule.connection.type = gateway_comms.msg.Connection.PUBLISHER
  req.flip_rule.gateway = args.gateway
  #req.node_name = "talker"
  #req.remapped_name = "dude"
  print ""
  print "== Request =="
  print ""
  print req
  print ""
  resp = flip(req)
  print "== Response =="
  print ""
  print resp

