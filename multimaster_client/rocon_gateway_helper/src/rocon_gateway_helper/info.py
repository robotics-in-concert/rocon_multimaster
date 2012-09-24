# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Yujin Robot, Daniel Stonier
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

import rospy
import rosgraph
import rosmaster
import rostopic
import rosnode
import rosservice
import itertools

def get_service_info(service_name):

    print service_name
    srvuri = rosservice.get_service_uri(service_name) 
    nodename = rosservice.get_service_node(service_name)
    print nodename
    master =rosgraph.Master(nodename)
    nodeuri = rosnode.get_api_uri(master,nodename)

    return (service_name, srvuri, nodeuri)

def get_topic_info(topic_name):
    name = rospy.get_name()

    topic_type, _1,_2 = rostopic.get_topic_type(topic_name)
    master =rosgraph.Master(name)
    pubs, _1,_2 = master.getSystemState()
    pubs = [x for x in pubs if x[0] == topic_name]
    apis = []

    if not pubs:
      raise Exception("Unknown topic %s",topic_name)

    for p in itertools.chain(*[l for x, l in pubs]):
      apis.append(rostopic.get_api(master,p))

    return (topic_name,topic_type,apis)

