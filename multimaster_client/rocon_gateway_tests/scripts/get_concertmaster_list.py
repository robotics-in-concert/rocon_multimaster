#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway_tests/LICENSE 
#


"""
  get_public_interfaces.py

  It shows the currently available concertmaster
"""

import roslib; roslib.load_manifest('rocon_gateway_tests')
import rospy
from gateway_comms.msg import *
from gateway_comms.srv import *

if __name__ == '__main__':

  rospy.init_node('get_concertmaster_list')

  try:
    s = rospy.ServiceProxy('/gateway/request',PublicHandler)
  
    req = PublicHandlerRequest()
    req.command = "post"
    req.list = ["getmembers","rocon:concertmasterlist",""]
    resp = s(req)
    
    print str(resp)
  except Exception as e:
    print str(e)
