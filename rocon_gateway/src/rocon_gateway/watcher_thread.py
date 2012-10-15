#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway/LICENSE 
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

import utils
from gateway_comms.msg import Rule, ConnectionType

##############################################################################
# Watcher
##############################################################################

class WatcherThread(threading.Thread):
    '''
    '''
    
    def __init__(self,gateway,watch_loop_period):
        threading.Thread.__init__(self)
        self._gateway = gateway
        self._master = gateway.master
        self._hub = gateway.hub
        #self._public_interface = gateway.public_interface
        self._flipped_interface = gateway.flipped_interface
        self._pulled_interface = gateway.pulled_interface
        self._watch_loop_rate = rospy.Rate(1.0/watch_loop_period)
        self.start()

    def run(self):
        while not rospy.is_shutdown():
            if self._gateway.is_connected:
                try:
                    connections = self._master.getConnectionState()
                except httplib.ResponseNotReady as e:
                    rospy.logwarn("Received ResponseNotReady from master api")
                    self._watch_loop_rate.sleep()
                    continue
                # Flipped Interface
                new_flips, lost_flips = self._flipped_interface.update(connections)
                # new_flips and lost_flips are RemoteRule lists with filled supplied name info from the master
                for connection_type in connections:
                    for flip in new_flips[connection_type]:
                        connection = self._master.generateConnectionDetails(flip.rule.type, flip.rule.name, flip.rule.node)
                        rospy.loginfo("Flipping to %s : %s"%(flip.gateway,utils.formatRule(connection.rule)))
                        self._hub.sendFlipRequest(flip.gateway, connection)
                    for flip in lost_flips[connection_type]:
                        rospy.loginfo("Unflipping to %s : %s"%(flip.gateway,utils.formatRule(flip.rule)))
                        self._hub.sendUnflipRequest(flip.gateway, flip.rule)
                # Public Interface
                self._gateway.updatePublicInterface(connections)

                # Pulled Interface
                for gateway in self._hub.listGateways():
                    if gateway == self._gateway.unique_name: #don't pull from self
                        continue
                    connections = self._hub.getRemoteConnectionState(gateway)
                    new_pulls, lost_pulls = self._pulled_interface.update(connections, gateway)
                    for connection_type in connections:
                        for pull in new_pulls[connection_type]:
                            for connection in connections[pull.rule.type]:
                                if connection.rule.name == pull.rule.name and \
                                   connection.rule.node == pull.rule.node:
                                    corresponding_connection = connection
                                    break
                            # Register this pull
                            existing_registration = self._pulled_interface.findRegistrationMatch(gateway, pull.rule.name, pull.rule.node, pull.rule.type)
                            if not existing_registration:
                                registration = utils.Registration(connection, gateway) 
                                new_registration = self._master.register(registration)
                                self._pulled_interface.registrations[registration.connection.rule.type].append(new_registration)
                        for pull in lost_pulls[connection_type]:
                            # Unregister this pull
                            existing_registration = self._pulled_interface.findRegistrationMatch(gateway, pull.rule.name, pull.rule.node, pull.rule.type)
                            if existing_registration:
                                self._master.unregister(existing_registration)
                                self._pulled_interface.registrations[existing_registration.connection.rule.type].remove(existing_registration)
            self._watch_loop_rate.sleep()
