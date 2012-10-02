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
  flip_service.py script

  It flips given services to given channels.(?)

  Usage   :
    rosrun rocon_gateway_tests flipout_service.py --clients <clients> ... --message  <servicename,srv_api,nodeuri> ...

   Service api and node uri for local service can be checked using get_service_info.py
"""
if __name__ == '__main__':

  parser = argparse.ArgumentParser(description='Process gateway request.')
  parser.add_argument('-c','--clients',metavar='<Client name>',type=str,nargs='+',help='Client\'s unique name on hub')
  parser.add_argument('-m','--message',metavar='<Service triple>',type=str,nargs='+',help='<Service triple>="<Service name>,<Service api>,<node uri>"')
  args = parser.parse_args()

  rospy.init_node('flipout_service')

  s = rospy.ServiceProxy('/gateway/request',PublicHandler)

  l = []
  l.append(str(len(args.clients)))
  l += args.clients
  l += args.message

  print "Serivces " + str(l)

  # Form a request message
  req = PublicHandlerRequest() 
  req.command = "flipout_service"
  req.list = l

  # Receive whether it is successful
  resp = s(req)

  # Print Result
  print resp

