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
  unregister_foreign_topic.py 
  
  It unregisters topics that is registered from public interface

  Usage   :
    rosrun rocon_gateway_tests unregister_foreign_topic.py -m <topicname,topictype,nodeuri> ...
  Example :
    rosrun rocon_gateway_tests unregister_foreign_topic.py -m /topic2,topic2type,node2uri

  topic_type and node uri for local topic can be checked using get_topic_info.py
"""
if __name__ == '__main__':

  parser = argparse.ArgumentParser(description='Process gateway request.')
  parser.add_argument('-m','--message',metavar='<Topic triple>',type=str,nargs='+',help='<Topic triple>="<topic name>,<topic type>,<node uri>"')
  args = parser.parse_args()

  rospy.init_node('unregister_foreign_topic')

  s = rospy.ServiceProxy('/gateway/request',PublicHandler)
  
  # all arguements are topicinfostring
  l = args.message
  print "Topics " + str(l)

  # Form a request message
  req = PublicHandlerRequest() 
  req.command = "unregister_foreign_topic"
  req.list = l

  # Receive whether it is successful
  resp = s(req)

  print resp
