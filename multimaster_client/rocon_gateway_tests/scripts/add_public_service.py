#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway_tests/LICENSE 
#

import roslib; roslib.load_manifest('rocon_gateway_tests')
import rospy
from rocon_gateway_helper import *
from gateway_comms.msg import *
from gateway_comms.srv import *
import argparse

"""
    add_public_service.py 
    
    It publicize local service to the centralised multimaster server

    Usage     :
        rosrun rocon_gateway_tests add_public_service.py --message <service_name,service_api,node uri> ...
    Example :
        rosrun rocon_gateway_tests add_public_service.py --message /add_two_ints,<service_api>,<nodeuri>

        Lookup    local service : 
            rosservice list
"""

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Process gateway request.')
    parser.add_argument('-m','--message',metavar='<Service triple>',type=str,nargs='+',help='<Service triple>="<Service name>,<Service api>,<node uri>"')
    args = parser.parse_args()

    rospy.init_node('add_public_service')

    s = rospy.ServiceProxy('/gateway/request',PublicHandler)

    
    # all arguements are service names
    l = args.message
    print "Service " + str(l)

    # Form a request message
    req = PublicHandlerRequest() 
    req.command = "add_public_service"
    req.list = l

    # Receive whether it is successful
    resp = s(req)

    # Print result
    print resp
