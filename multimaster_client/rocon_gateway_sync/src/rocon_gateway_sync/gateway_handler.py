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

class GatewayHandler(rosgraph.xmlrpc.XmlRpcHandler):
    '''
    Sets up the handlers for the xmlrpc node. Any public
    method defined here will be associated with the 
    xmlrpc server.
    '''
    def __init__(self):
        '''
        Constructor
        '''
        pass
    
    ##########################################
    # XmlRpcHandler Callbacks    
    ##########################################    
    def _ready(self, uri):
        """
        callback into handler to inform it of XML-RPC URI
        """
        #print "Xmlrpc Handler: ready"
        pass

    def _shutdown(self, reason):
        """
        callback into handler to inform it of shutdown
        """
        #print "Xmlrpc Handler: shutdown"
        pass


    ##########################################
    # Xmlrpc Api
    ##########################################
    
    def list(self):
        return "list"
    
    def handle_registration(self):
        return "Flip requested ros api from the remote gateway to the local master"

    def handle_request(self):
        return "Flip exposed ros api from the local master to the remote gateway"
