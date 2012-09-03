#!/usr/bin/env python
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

import os
import sys
import time
from optparse import OptionParser
import roslib; roslib.load_manifest('rocon_gateway_sync')
import rospy
import rosgraph
import rocon_gateway_sync

##############################################################################
# Testing
##############################################################################

# > roscore
# > rosrun roscpp_tutorials talker
# > show_node_xmlrpc_uri /talker

# Copy the output of the last command, we'll use it below in <node_xmlrpc_uri>

# > roscore --port 11312
# export ROS_MASTER_URI=http://localhost:11312; ./register_publisher <node_xmlrpc_uri>

# This neatly transfers across a publisher to the remote system.

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    parser = OptionParser(usage="usage: %prog <node_xmlrpc_uri>")
    options, args = parser.parse_args()
        
    rospy.init_node('dude')
    if not args:
        parser.error("You must specify a node xmlrpc uri for a node with the talker (/chatter) enlisted on another master.")
    else:
        node_xmlrpc_uri = args[0]
        name = rospy.get_name()
    if rosgraph.is_master_online:
        print
        print("******************* General Information *************************")
        print
        print("Node name: "+name)
        master = rosgraph.Master(name)
        print
        try:
           print("Registering /chatter with type std_msgs/String from xmlrpc node "+node_xmlrpc_uri)
           master.registerPublisher("/chatter", "std_msgs/String", node_xmlrpc_uri)
        except:
           print("Failed.")
        print
        print("*****************************************************************")
        print
    else:
        print "Master is not online"
