#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway/LICENSE 
#

##############################################################################
# Imports
##############################################################################

import roslib; roslib.load_manifest('rocon_gateway')
import rospy
import rosservice
import rostopic
import threading
import httplib
from gateway_comms.msg import Rule

##############################################################################
# Watcher
##############################################################################

class WatcherThread(threading.Thread):
    '''
    '''
    
    def __init__(self,gateway,watch_loop_period):
        # init thread
        threading.Thread.__init__(self)
        self.gateway = gateway
        self.master = gateway.master
        self.hub = gateway.hub
        self.public_interface = gateway.public_interface
        self.flipped_interface = gateway.flipped_interface
        self.watch_loop_rate = rospy.Rate(1.0/watch_loop_period)
        self.start()

    def run(self):
        while not rospy.is_shutdown():
            if self.gateway.is_connected:
                try:
                    connections = self.master.getConnectionState()
                except httplib.ResponseNotReady as e:
                    rospy.logwarn("Received ResponseNotReady from master api")
                    self.watch_loop_rate.sleep()
                    continue
                # Flipped Interface
                new_flips, lost_flips = self.flipped_interface.update(connections)
                # new_flips and lost_flips are RemoteRule lists with filled supplied name info from the master
                for connection_type in connections:
                    for flip in new_flips[connection_type]:
                        xmlrpc_uri = self.master.lookupNode(flip.rule.node)
                        if connection_type == Rule.PUBLISHER or connection_type == Rule.SUBSCRIBER:
                            type_info = rostopic.get_topic_type(flip.rule.name)[0] # message type
                        elif connection_type == Rule.SERVICE:
                            type_info = rosservice.get_service_uri(flip.rule.name)
                        self.hub.sendFlipRequest(flip, type_info, xmlrpc_uri )
                    for flip in lost_flips[connection_type]:
                        self.hub.sendUnFlipRequest(flip)
                # Public Interface
                self.gateway.updatePublicInterface(connections)
            self.watch_loop_rate.sleep()
