#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway_tests/LICENSE 
#

import roslib; roslib.load_manifest('rocon_gateway_tests')
import rospy
from rocon_gateway_helper import *
from gateway_comms.msg import *
from gateway_comms.srv import *
import argparse

"""
    add_public_topic.py 
    
    It publicize local topic to the centralised multimaster server

    Usage     :
        rosrun rocon_gateway_tests add_public_topic.py -m <topic_name,topictype,node_uri> ...
    Example :
        rosrun rocon_gateway_tests add_public_topic.py -m /chatter,std_msgs/String,<nodeuri> ...

        Lookup    local topic : 
            rostopic list
"""
if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Process gateway request.')
    parser.add_argument('-m','--message',metavar='<Topic triple>',type=str,nargs='+',help='<Topic triple>="<topic name>,<topic type>,<node uri>"')
    args = parser.parse_args()

    rospy.init_node('add_public_topic')

    s = rospy.ServiceProxy('/gateway/request',PublicHandler)
    
    # all arguements are topic names
    l = args.message
    print "Topics " + str(l)

    # Form a request message
    req = PublicHandlerRequest() 
    req.command = "add_public_topic"
    req.list = l

    # Receive whether it is successful
    resp = s(req)

    print resp

