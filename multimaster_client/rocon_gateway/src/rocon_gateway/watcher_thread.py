#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway/LICENSE 
#

import rospy
import threading
from gateway_comms.msg import Connection

class WatcherThread(threading.Thread):

    def __init__(self,gateway):
        # init thread
        threading.Thread.__init__(self)
        self.gateway = gateway
        self.master = gateway.master
        self.cv = self.master.cv
        self.pubs = self.master.pubs_node
        self.public_interface = gateway.public_interface
    
    def run(self):
        print "Thread Started"
        while not rospy.is_shutdown():
            self.cv.acquire()

            # self.checkRemoteList()
            if self.gateway.is_connected: 
                # 1. Check all remove interfaces are still in redis server, if it is gone, unregister it 
                #self.pollServer()
                # 2. Check all local public interfaces are still valid
                self.checkPublicInterfaces()
            self.cv.release()
            rospy.sleep(3.0)

    def pollServer(self):
        remotelist = self.gateway.getRemoteLists()

        for master in remotelist:
            print str(remotelist[master])

    def checkPublicInterfaces(self):
        pubs, subs, srvs = self.master.getSystemState() 
        self.update(Connection.PUBLISHER,pubs)
        self.update(Connection.SUBSCRIBER,subs)
        self.update(Connection.SERVICE,srvs)

    def update(self,identifier,list):
        # unadvertise from public interface if a topic disappears from the local master
        for string in self.public_interface.interface[identifier]:
            name, _, node_uri = string.split(",")
            still_exist = False
            try:
                llist = [x[1] for x in list if x[0] == name]
  
                # all nodes are gone.
                uris = [self.master.lookupNode(p) for p in llist[0]]
                still_exist = node_uri in uris
            except:
                still_exist = False
              
            # if it is not exist anymore, remove it from public interface
            if not still_exist:
                self.gateway.unadvertise([string])

        # CURRENTLY DISABLED (work in progress)
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
  
        # add/remove named interfaces to flipped list as necessary
        for x in list:
            name = x[0]
            clients, non_clients = self.gateway.getFlippedClientList(identifier, name)
            self.gateway.addFlippedInterfaceByName(identifier,clients,name)
            self.gateway.removeFlippedInterfaceByName(identifier,non_clients,name)

        

"""
  polling thread should do...
  1. unregister the unavailable remote topics/services
  2. remove the unavailable topics/services from public list
"""
