#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway_helper/LICENSE 
#

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

