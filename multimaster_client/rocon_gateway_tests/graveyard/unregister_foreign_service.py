#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway_tests/LICENSE 
#

"""
  unregister_foreign_service.py 
  
  It unregisters services that is registered from public interface

  Usage   :
    rosrun rocon_gateway_tests unregister_foreign_service.py -m <servicename,srv_api,nodeuri> ...
  Example :
    rosrun rocon_gateway_tests unregister_foreign_service.py -m /service1,service1api,node1uri /service2,service2api,node2uri

    Service api and node uri for local service can be checked using get_service_info.py
"""
import roslib; roslib.load_manifest('rocon_gateway_tests')
import rospy
from gateway_comms.msg import *
from gateway_comms.srv import *
import argparse

if __name__ == '__main__':

  parser = argparse.ArgumentParser(description='Process gateway request.')
  parser.add_argument('-m','--message',metavar='<Service triple>',type=str,nargs='+',help='<Service triple>="<Service name>,<Service api>,<node uri>"')
  args = parser.parse_args()

  rospy.init_node('unregister_foreign_service')

  s = rospy.ServiceProxy('/gateway/request',PublicHandler)
  
  # all arguements are service names
  l = args.message
  print "Services " + str(l)

  # Form a request message
  req = PublicHandlerRequest() 
  req.command = "unregister_foreign_service"
  req.list = l

  # Receive whether it is successful
  resp = s(req)

  # Print result
  print resp

