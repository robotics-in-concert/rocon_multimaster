#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway/LICENSE 
#

import rospy
import threading
from gateway_comms.msg import Connection

class WatcherThread(threading.Thread):
    '''
    '''
    
    def __init__(self,gateway):
        # init thread
        threading.Thread.__init__(self)
        self.gateway = gateway
        self.master = gateway.master
        self.cv = self.master.cv
        self.pubs = self.master.pubs_node
        self.public_interface = gateway.public_interface
        self.flipped_interface = gateway.flipped_interface

    
        self.start()

    def run(self):
        while not rospy.is_shutdown():
            self.cv.acquire()

            if self.gateway.is_connected: 
                self._checkLocalConnections()
            self.cv.release()
            rospy.sleep(3.0)

    def _checkLocalConnections(self):
        '''
          Checks the local master and collates a list of all
          current connections. We use this to check against all 
          our rules to determine whether any connections have
          become available or unavailable since the last lookup.
          
          @todo : prune pubs and subs and collate action client 
                  and server lists  
        '''
        publishers, subscribers, services = self.master.getSystemState()
        actions = [] # todo : create and prune pubs/subs
        self._updateFlips(publishers, subscribers, services, actions)
        
        # (PK) ^ I've implemented functions to get action_server, action_client in the master api

        connections = self.master.getConnectionState()
        self._updatePublicInterface(connections)

    def _updateFlips(self, publishers, subscribers, services, actions):
        '''
          Process the list of local connections and check against 
          the current rules and patterns for flips. If a connection 
          has become (un)available take appropriate action.
          
          @param publishers, subscribers, services, actions
          @type list of ??? : 
        '''
        # 0) prune rules/patterns that apply to gateways no longer on the hub
        existing_flips = set()
        new_flips = set()
        for topic, node_list in publishers:
            #print "Topic: %s"%topic
            #print "Nodelist:"
            for node in node_list:
                flip = self.flipped_interface.isFlipped(Connection.PUBLISHER,topic,node)
                if flip:
                    existing_flips.add(flip)
                else:
                    # check rules
                    matched_flip = self.flipped_interface.matchesRule(Connection.PUBLISHER,topic,node)
                    if matched_flip:
                        new_flips.add(matched_flip)
                    # check patterns
                
        # 1) compare with non-active flip rules & patterns, flip if there's a hit
        for flip in new_flips:
            print "New flip"
            continue
        
        # 2) compare with currently flipped interfaces checking for one that has disappeared
        disappeared_flips = self.flipped_interface.flipped - existing_flips

    def _updatePublicInterface(self, connections):
        '''
          Process the list of local connections and check against 
          the current rules and patterns for flips. If a connection 
          has become (un)available take appropriate action.
          
          @param connections
          @type dictionary of connections 
        '''

        for connection_type in connections:
            allowed_connections = self.public_interface.allowedConnections(connections[connection_type])
            
            # this has both connections that have disappeared or are no longer allowed
            unadvertise_connections = self.public_interface.public - allowed_connections
            advertise_new_connections = allowed_connections - self.public_interface.public

            for connection in advertise_new_connections:
                self.gateway.advertiseConnection(connection)

            for connection in unadvertise_connections:
                self.gateway.unadvertiseConnection(connection)
    
    def update(self, type, connections):
        # CURRENTLY DISABLED (work in progress)
        # unadvertise from public interface if a topic disappears from the local master
        # for string in self.public_interface.interface[identifier]:
        #     name, _, node_uri = string.split(",")
        #     still_exist = False
        #     try:
        #         llist = [x[1] for x in list if x[0] == name]
        #
        #         # all nodes are gone.
        #         uris = [self.master.lookupNode(p) for p in llist[0]]
        #         still_exist = node_uri in uris
        #     except:
        #         still_exist = False
        #       
        #     # if it is not exist anymore, remove it from public interface
        #     if not still_exist:
        #         self.gateway.unadvertise([string])
        #
        # # add/remove named interfaces to public list as necessary
        # for x in list:
        #     name = x[0]
        #     if self.gateway_sync.allowInterfaceInPublic(identifier, name):
        #         # check if any new publishers are available
        #         self.gateway_sync.addPublicInterfaceByName(identifier, name)
        #         self.dumped_interface[identifier].add(name)
        #     else:
        #         # this interface has been dumped in the past, and is no longer needed
        #         if name in self.dumped_interface[identifier]:
        #             self.gateway.removePublicInterfaceByName(identifier, name)
        #             self.dumped_interface[identifier].remove(name)
  
        # DJS: CURRENTLY DISABLED (work in progress)
        # add/remove named interfaces to flipped list as necessary
        # for x in list:
        #     name = x[0]
        #     clients, non_clients = self.gateway.getFlippedClientList(identifier, name)
        #     self.gateway.addFlippedInterfaceByName(identifier,clients,name)
        #     self.gateway.removeFlippedInterfaceByName(identifier,non_clients,name)
        pass
