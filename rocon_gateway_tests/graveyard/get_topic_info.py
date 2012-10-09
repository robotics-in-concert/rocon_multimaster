#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway_tests/LICENSE 
#

import roslib; roslib.load_manifest('rocon_gateway_tests')
import rospy
import rostopic
import rosnode
import rosgraph
import itertools
import sys
import argparse

"""
  get_topic_info.py

  It prints general topic information including 
    topic name, topic type, and topic nodes' uri

  Usage : 
    rosrun rocon_gateway_tests get_service_info.py --topic <topic_name>
  Ex    :
    rosrun rocon_gateway_tests get_topic_info.py --topic /chatter
"""

if __name__ == '__main__':

  parser = argparse.ArgumentParser(description='Provide a topic information and triple.')
  parser.add_argument('-t','--topic',metavar='<topic name>',type=str,help='Ex : /chatter')
  args = parser.parse_args()

  rospy.init_node('get_topic_info')
  name = rospy.get_name()
  master = rosgraph.Master(name)


  try:
    infolist = []
    topic_name = args.topic
    topic_type, _1,_2 = rostopic.get_topic_type(topic_name)
    pubs, _1,_2 = master.getSystemState()
    pubs = [x for x in pubs if x[0] == topic_name]

    if not pubs:
      raise Exception("Unknown topic %s",topic_name)

    print "== Topic =="
    print " - Name     : " + topic_name
    print " - Type     : " + topic_type
    print " - Node Uri : " 

    # Print Publisher uris
    for p in itertools.chain(*[l for x, l in pubs]):
      api = rostopic.get_api(master,p)
      print "              " + str(api)
      info = topic_name + "," + topic_type + "," + api
      infolist.append(info)
      
    # Print info strings
    print " - Concat   : " 
    for info in infolist:
      print "              " + info


  except Exception as e:
    print str(e)

