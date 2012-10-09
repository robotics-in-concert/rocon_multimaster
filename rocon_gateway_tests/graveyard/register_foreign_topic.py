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
  register_foreign_topic.py 
  
  It registers topic that publically available.

  Usage   :
    rosrun rocon_gateway_tests register_foreign_topic.py -m <topic_name,topic_type,nodeuri> ...
  Example :
    rosrun rocon_gateway_tests register_foreign_topic.py -m /topic1,topic1type,node1uri /topic2,topic2type,node2uri 

    Available public topics can be checked using get_remote_list.py 
    It drops registration if it tries to register local topic
"""
if __name__ == '__main__':

  parser = argparse.ArgumentParser(description='Process gateway request.')
  parser.add_argument('-m','--message',metavar='<Topic triple>',type=str,nargs='+',help='<Topic triple>="<topic name>,<topic type>,<node uri>"')
  args = parser.parse_args()

  rospy.init_node('register_public_topic')

  s = rospy.ServiceProxy('/gateway/request',PublicHandler)
  

  l = args.message
  print "Topics " + str(l)

  # Form a request message
  req = PublicHandlerRequest() 
  req.command = "register_foreign_topic"
  req.list = l

  # Receive whether it is successful
  resp = s(req)

  # Print result
  print resp

