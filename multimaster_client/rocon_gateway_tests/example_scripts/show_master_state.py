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
import xmlrpclib
import roslib; roslib.load_manifest('rocon_gateway_sync')
import rospy
import rosgraph

##############################################################################
# Launching
##############################################################################

if __name__ == '__main__':
    rospy.init_node('dude')
    if rosgraph.is_master_online:
        name = rospy.get_name()
        print
        print("******************* General Information *************************")
        print
        print("Node name: "+name)
        master = rosgraph.Master(name)
        master_uri = master.getUri()
        print("Master uri: "+master_uri)
        [publishers, subscribers, services] = master.getSystemState()
        print
        print("************************ Master State ***************************")
        print
        print("Publishers")
        for (publisher, nodes) in services:
            print "  "+publisher+" ["+",".join(node for node in nodes)+"]"
        print
        print("Subscribers")
        for (subscriber, nodes) in services:
            print "  "+subscriber+" ["+",".join(node for node in nodes)+"]"
        print
        print("Services")
        for (service, nodes) in services:
            print "  "+service+" ["+",".join(node for node in nodes)+"]"
        print
        print("*****************************************************************")
        print
        print("Other useful rosgraph masterapi info comes from getTopicTypes")
        print("and getPublishedTopics")
        print
        print("*****************************************************************")
        print
    else:
        print "Master is not online"
