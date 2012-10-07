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
  register_foreign_service.py 
  
  It registers services that is publicly available.

  Usage   :
    rosrun rocon_gateway_tests register_foreign_service.py --message <servicename,srv_api,nodeuri> ...
  Example :
    rosrun rocon_gateway_tests register_foreign_service.py --message /service1,service1api,node1uri /service2,service2api,node2uri

    Available public services can be checked using get_remote_list.py 
    It drops registration if it tries to register local service
"""

if __name__ == '__main__':

  parser = argparse.ArgumentParser(description='Process gateway request.')
  parser.add_argument('-m','--message',metavar='<Service triple>',type=str,nargs='+',help='<Service triple>="<Service name>,<Service api>,<node uri>"')
  args = parser.parse_args()

  rospy.init_node('register_foreign_service')

  s = rospy.ServiceProxy('/gateway/request',PublicHandler)
  
  # all arguements are service info strings
  l = args.message
  print "Service " + str(l)

  # Form a request message
  req = PublicHandlerRequest() 
  req.command = "register_foreign_service"
  req.list = l

  # Receive whether it is successful
  resp = s(req)

  # Print result
  print resp

