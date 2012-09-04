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
    parser = OptionParser(
                   usage="usage: %prog [OPTIONS]\n\n\
%prog launches a hub enabling ros multimaster:\n\
  1. Launches a central redis server for gateway information and interactions\n\
  2. Optionally advertises a zeroconf service appropriate to your platform\n\",
                   epilog="See: http://www.ros.org/wiki/rocon_multimaster for details\n")
    parser.add_option("-p", "--port", dest="port", default=0,
                    help="Port on which to launch the redis server.",
                    action="store")
    options, args = parser.parse_args()
    
    is_ros_environment = false;
    try:
        import roslib; roslib.load_manifest('rocon_gateway')
        import rospy
        import rosgraph
        import rocon_gateway
        is_ros_environment = true
        rospy.init_node('hub')
        rospy.loginfo("Ros environment detected")
    except ImportError:
        print("No ros environment detected.")

#    if rosgraph.is_master_online:
#        print
#        print("******************* General Information *************************")
#        print
#        print("*****************************************************************")
#        print
#    else:
#        print "Master is not online"
