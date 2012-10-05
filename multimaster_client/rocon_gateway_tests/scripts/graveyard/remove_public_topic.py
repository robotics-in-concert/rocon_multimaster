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
  remove_public_topic.py 
  
  It stop publicizing local topic to the centralised multimaster server

  Usage   :
    rosrun rocon_gateway_tests remove_public_topic.py -m <topicname,topictype,nodeuri> ...

  Lookup topic info string : 
    get_topic_info.py
"""
if __name__ == '__main__':

  parser = argparse.ArgumentParser(description='Process gateway request.')
  parser.add_argument('-m','--message',metavar='<Topic triple>',type=str,nargs='+',help='<Topic triple>="<topic name>,<topic type>,<node uri>"')
  args = parser.parse_args()

  rospy.init_node('remove_public_topic')

  s = rospy.ServiceProxy('/gateway/request',PublicHandler)
  
  l = args.message
  print "Topics " + str(l)

  # Form a request message
  req = PublicHandlerRequest() 
  req.command = "remove_public_topic"
  req.list = l

  # Receive whether it is successful
  resp = s(req)
                                     
  # Print result
  print resp
