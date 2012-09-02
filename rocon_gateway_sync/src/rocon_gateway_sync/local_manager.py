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

import roslib; roslib.load_manifest('rocon_gateway_sync')
import rosgraph

class LocalManager(object):
    '''
    Abstraction layer over the xmlrpc interface to the local master.
    '''
    def __init__(self):
        '''
        Constructor
        '''
        self.master_uri = rosgraph.get_master_uri()
        
        ns = rosgraph.names.get_ros_namespace() # ns in which this node resides, typically '/'
        anon_name = rosgraph.names.anonymous_name('gateway') # creates a unique name, e,g, gateway_0258198135

        self.master = rosgraph.masterapi.Master(rosgraph.names.ns_join(ns, anon_name), master_uri=self.master_uri)
        #self.cb = cb
        self.type_cache = {}
        self.subs = {}
        self.pubs = {}
        self.srvs = {}
    
    def get_topic_type(self, query_topic):
        query_topic = self.resolve(query_topic)
        if query_topic in self.type_cache:
            return self.type_cache[query_topic]
        else:
            for topic, topic_type in self.master.getTopicTypes():
                self.type_cache[topic] = topic_type
            if query_topic in self.type_cache:
                return self.type_cache[query_topic]
            else:
                return "*"
            
    def resolve(self, topic):
        ns = rosgraph.names.namespace(self.master.caller_id)
        return rosgraph.names.ns_join(ns, topic)
