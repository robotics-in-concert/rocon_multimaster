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
  flip_publisher.py script <gateway>
  
  Usage   :
    rosrun rocon_gateway_tests flip_publisher.py
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
  req.type = gateway_comms.msg.Connection.PUBLISHER
  req.gateway = args.gateway
  req.name = "/chatter"
  #req.node_name = "talker"
  #req.remapped_name = "dude"
  print ""
  print "== Request =="
  print req
  print ""
  resp = flip(req)
  print "== Response =="
  print resp

