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
  flip_topic.py script

  It flips given topics to given channels.(?)

  Usage   :
    rosrun rocon_gateway_tests flipout_topic.py --clients <clients> ... --message  <topicname,topictype,nodeuri> ...

    topic_type and node uri for local topic can be checked using get_topic_info.py
"""

if __name__ == '__main__':

  parser = argparse.ArgumentParser(description='Process gateway request.')
  parser.add_argument('-c','--clients',metavar='<Client name>',type=str,nargs='+',help='Client\'s unique name on hub')
  parser.add_argument('-m','--message',metavar='<Topic triple>',type=str,nargs='+',help='<Topic triple>="<topic name>,<topic type>,<node uri>"')
  args = parser.parse_args()

  rospy.init_node('flipout_topic')

  s = rospy.ServiceProxy('/gateway/request',PublicHandler)
  
  l = []
  l.append(str(len(args.clients)))
  l += args.clients
  l += args.message

  print "Topics " + str(l)

  # Form a request message
  req = PublicHandlerRequest() 
  req.command = "flipout_topic"
  req.list = l

  # Receive whether it is successful
  resp = s(req)

  # Print Result
  print resp

