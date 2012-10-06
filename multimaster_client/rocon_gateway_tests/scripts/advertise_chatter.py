#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway_tests/LICENSE 
#

import roslib; roslib.load_manifest('rocon_gateway_tests')
import rospy
from gateway_comms.msg import *
from gateway_comms.srv import *

if __name__ == '__main__':

  rospy.init_node('flip_publisher')

  flip = rospy.ServiceProxy('/gateway/advertise',Advertise)
  
  # Form a request message
  req = AdvertiseRequest()
  req.rule.type = gateway_comms.msg.Connection.PUBLISHER
  req.rule.name = "/chatter"
  req.cancel = False
  print ""
  print "== Request =="
  print req
  print ""
  resp = flip(req)
  print "== Response =="
  print resp

