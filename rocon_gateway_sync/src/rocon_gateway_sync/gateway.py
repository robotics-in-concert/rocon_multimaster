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
#  * Neither the name of I Heart Engineering nor the names of its
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
import threading
from SimpleXMLRPCServer import SimpleXMLRPCServer
import roslib; roslib.load_manifest('rocon_gateway_sync')
import rospy
import rosgraph
from std_msgs.msg import Empty

class GatewayRemoteInterface(threading.Thread):
    '''
    Sets up the remote xmlrpc server interface that remote systems use to
    interact with this gateway.
    '''
    def __init__(self):
        threading.Thread.__init__(self)
        '''
        Sets up the xmlrpc server.
        '''
        self.port = 0 # will be set later when the socket is created 
        # Xmlrpc server interface
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind(("",0))  # The empty string is equivalent to socket.INADDR_ANY (any network interface on the computer)
        self.address, self.port = s.getsockname()
        s.close()
        self._server = SimpleXMLRPCServer(("", self.port)) # The empty hostname string is equivalent to socket.INADDR_ANY (any network interface on the computer)
        self._server.register_introspection_functions() # allows a client to do print s.system.listMethods()
        self._server.register_function(self.list, 'list')
        self._server.register_function(self.handle_registration, 'handle_registration')
        self._server.register_function(self.handle_request, 'handle_request')

        print("Bind address: " + rosgraph.network.get_bind_address())
        print("Local address: " + rosgraph.network.get_local_address())
        print("Xmlrpc server ["+self.address+":"+str(self.port)+"]")

    def list(self):
        return "list"
    
    def handle_registration(self):
        return "Flip requested ros api from the remote gateway to the local master"

    def handle_request(self):
        return "Flip exposed ros api from the local master to the remote gateway"

    def run(self):
        self._server.serve_forever()        

    def stop(self):
        '''
          Shutdown the xmlrpc server and thread that it runs in.
        '''
        self._server.shutdown()
        if self.isAlive():
            self.join()
                 

    
class Gateway(object):
    '''
    The gateway between ros systems.
    '''
    def __init__(self):
        '''
        Creates a new gateway interface
        '''
        self.whitelist=[]
        self.blacklist=[]
        self.ros_master_uri = rosgraph.rosenv.get_master_uri()
        self._register_subscriber = rospy.Subscriber("register", Empty, self.register_cb)
        
        self.remote_interface=GatewayRemoteInterface()
        self.remote_interface.start()
        
        print("Created gateway ["+self.ros_master_uri+"]")
    
    def shutdown(self):
        self.remote_interface.stop()
        
    def register_cb(self, data):
        print("Registering local api [" + self.ros_master_uri + "] to the remote gateway")
        
