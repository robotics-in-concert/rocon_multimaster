#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway_tests/LICENSE 
#


import roslib; roslib.load_manifest('rocon_gateway_tests')
import rospy
import rosservice
import rosnode
import rosgraph
import sys
import argparse

"""
  get_service_info.py

  It prints general service information including 
    service name, service uri, and service node uri

  Usage : 
    rosrun rocon_gateway_tests get_service_info.py --service <service_name>
    rosrun rocon_gateway_tests get_service_info.py --service /add_two_ints
"""

if __name__ == '__main__':

  parser = argparse.ArgumentParser(description='Provides a Service information and triple.')
  parser.add_argument('-s','--service',metavar='<Service name>',type=str,help='Ex : /add_two_ints')
  args = parser.parse_args()

  rospy.init_node('get_service_info')
  name = rospy.get_name()
  master = rosgraph.Master(name)

  try:
    service_name = args.service
    srvuri = rosservice.get_service_uri(service_name) 
    nodename = rosservice.get_service_node(service_name)
    nodeuri = rosnode.get_api_uri(master,nodename)

    print "== Service =="
    print " - Name     : " + service_name
    print " - Uri      : " + srvuri
    print " - Node Uri : " + nodeuri

    info = service_name + "," + srvuri + "," + nodeuri
    print " - Concat   : " + info

  except Exception as e:
    print str(e)


