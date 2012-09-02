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

import socket
import time
import roslib; roslib.load_manifest('rocon_gateway_sync')
import rospy
import rosgraph
from std_msgs.msg import Empty

from .gateway_handler import GatewayHandler
from .local_manager import LocalManager

class GatewaySync(object):
    '''
    The gateway between ros systems.
    '''
    def __init__(self):
        '''
        Creates a new gateway interface
        '''
        # Parameters
        self.port = rospy.get_param('~port',0)
        
        self.whitelist=[]
        self.blacklist=[]
        
        self.local_manager = LocalManager()
        
        self.handler = GatewayHandler()
        self.node = rosgraph.xmlrpc.XmlRpcNode(port=self.port,rpc_handler=self.handler,on_run_error=self._xmlrpc_node_error_handler) # Can also pass port and run_error handlers
        self.node.start()
        
        # poll for initialization
        timeout = time.time() + 5
        while time.time() < timeout and self.node.uri is None and not rospy.is_shutdown():
            time.sleep(0.01)
        if self.node.uri is None:
            # Some error handling here
            return
            
        # Uri is determined by lockup in order (rosgraph.xmlrpc.XmlRpcNode 
        #   1) ROS_IP/ROS_HOSTNAME 
        #   2) hostname (if not localhost) 
        #   3) rosgraph.network.get_local_address()
        self.uri = self.node.uri
        # could probably update self.port here if we wanted to (only necessary if we set port = 0 (ANY))

        print("Created gateway")
        print("  Node ["+self.uri+"]")
    
    def shutdown(self):
        self.node.shutdown("called shutdown")
        
    def _xmlrpc_node_error_handler(self, e):
        '''
        Handles exceptions of type 'Exception' thrown by the xmlrpc node.
        '''
        # Reproducing the xmlrpc node logic here so we can catch the exception
        if type(e) == socket.error:
            (n,errstr) = e
            #if n == 98: # can't start the node because the port is already used
                # save something to that the class can look up the problem later
        else:
            print("Xmlrpc Node Error")
    