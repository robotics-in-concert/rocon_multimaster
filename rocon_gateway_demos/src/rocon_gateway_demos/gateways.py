#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway_tests/LICENSE 
#

##############################################################################
# Imports
##############################################################################

import roslib; roslib.load_manifest('rocon_gateway_demos')
import rospy
import rocon_gateway
from gateway_msgs.srv import *

##############################################################################
# Functions
##############################################################################

def findFirstRemoteGateway():
    '''
      Parses the remote gateway list to find a gateway to use for testing.
      
      It's a dumb hack to make testing quite convenient.
      
      @return gateway string name
      @rtype string
    '''
    remote_gateway_info = rospy.ServiceProxy('/gateway/remote_gateway_info',RemoteGatewayInfo)
    req = RemoteGatewayInfoRequest()
    req.gateways = []
    resp = remote_gateway_info(req)
    if len(resp.gateways) == 0:
        raise rocon_gateway.GatewayError("no remote gateways available")
    else:
        return resp.gateways[0].name

