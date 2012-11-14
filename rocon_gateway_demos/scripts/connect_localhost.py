#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway_demos/LICENSE 
#

import roslib; roslib.load_manifest('rocon_gateway_demos')
import rospy
import gateway_msgs.srv
from gateway_msgs.srv import ConnectHub
import argparse

"""
  Tests the ros service handle (/gateway/connect_hub) for connecting
  a gateway to a hub.
  
  Usage:
    1 > roslaunch rocon_gateway_demos buccaneer_hub.launch
    2a> roslaunch rocon_gateway_demos buccaneer_gateway_ros_api.launch
    2b> rosrun rocon_gateway_demos connect_localhost.py
"""

if __name__ == '__main__':

  parser = argparse.ArgumentParser(description='Make a connection to a localhost hub by service call')

  rospy.init_node('connect_localhost')

  connect = rospy.ServiceProxy('/gateway/connect_hub',ConnectHub)
  
  # Form a request message
  req = gateway_msgs.srv.ConnectHubRequest() 
  req.uri = "http://localhost:6380"
  print ""
  print "== Request =="
  print ""
  print req
  print ""
  resp = connect(req)
  print "== Response =="
  print ""
  print resp

