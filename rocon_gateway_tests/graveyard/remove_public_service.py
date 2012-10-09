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
  remove_public_service.py 
  
  It stop publicizing local service to the centralised multimaster server

  Usage   :
    rosrun rocon_gateway_tests remove_public_service.py -m <service name,service api,node uri> ...
  Example :
    rosrun rocon_gateway_tests remove_public_service.py -m /service1,service1api,nodeuri

  Lookup  service info string : 
    get_service_info.py
"""
if __name__ == '__main__':

  parser = argparse.ArgumentParser(description='Process gateway request.')
  parser.add_argument('-m','--message',metavar='<Service triple>',type=str,nargs='+',help='<Service triple>="<Service name>,<Service api>,<node uri>"')
  args = parser.parse_args()

  rospy.init_node('remove_public_topic')

  s = rospy.ServiceProxy('/gateway/request',PublicHandler)
  
  # all arguements are service names
  l = args.message
  print "Services " + str(l)

  # Form a request message
  req = PublicHandlerRequest() 
  req.command = "remove_public_service"
  req.list = l

  # Receive whether it is successful
  resp = s(req)

  # Print result
  print resp
