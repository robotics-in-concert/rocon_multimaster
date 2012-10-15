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
        self.trigger_update = False
        self._trigger_shutdown = False
        self._gateway = gateway
        self._master = gateway.master
        self._hub = gateway.hub
        #self._public_interface = gateway.public_interface
        self._flipped_interface = gateway.flipped_interface
        self._pulled_interface = gateway.pulled_interface
        self._watch_loop_period = rospy.Duration(watch_loop_period)
        self._last_loop_timestamp = rospy.Time.now()
        self._internal_sleep_period = rospy.Duration(0,200000000) # 200ms
        self.start()
        
    def shutdown(self):
        '''
          Called from the main program to shutdown this thread.
        '''
        self._trigger_shutdown = True
        self._trigger_update = True # causes it to interrupt a sleep and drop back to check shutdown condition
        self.join() # wait for the thread to finish
        
    def run(self):
        '''
          The watcher thread - monitors both the local master's system state (list of connections)
          and the various rules to make sure rules and existing connections or flips are in sync.
        '''
        while not rospy.is_shutdown() and not self._trigger_shutdown:
            if self._gateway.is_connected:
                try:
                    connections = self._master.getConnectionState()
                except httplib.ResponseNotReady as e:
                    rospy.logwarn("Received ResponseNotReady from master api")
                    self._sleep()
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
                for gateway in self._hub.listRemoteGatewayNames():
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
            self._sleep()


    def _sleep(self):
        '''
          Internal interruptible sleep loop to check for shutdown and update triggers.
          This lets us set a really long watch_loop update if we wish.
        '''
        while not rospy.is_shutdown() and not self.trigger_update and (rospy.Time.now()-self._last_loop_timestamp < self._watch_loop_period):
            rospy.sleep(self._internal_sleep_period)
        self.trigger_update = False
        self._last_loop_timestamp = rospy.Time.now()
