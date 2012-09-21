#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Yujin Robot, Jihoon Lee
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Yujin Robot nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib; roslib.load_manifest('rocon_gateway_tests')
import rospy
import rostopic
import rosnode
import rosgraph
import itertools
import sys

"""
  get_topic_info.py

  It prints general topic information including 
    topic name, topic type, and topic nodes' uri

  Usage : 
    rosrun rocon_gateway_tests get_service_info.py <topic_name>
  Ex    :
    rosrun rocon_gateway_tests get_topic_info.py /chatter
"""

if __name__ == '__main__':

  rospy.init_node('get_topic_info')
  name = rospy.get_name()
  master = rosgraph.Master(name)

  if len(sys.argv) != 2:
    print "Usage :  rosrun rocon_gateway_tests get_topic_info.py <topic_name>"
    sys.exit()

  try:
    infolist = []
    topic_name = sys.argv[1]
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

