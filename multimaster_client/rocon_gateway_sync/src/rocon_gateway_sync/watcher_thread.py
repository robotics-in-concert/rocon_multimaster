#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway_sync/LICENSE 
#

import rospy
import threading
import rosmaster
import rosnode
import rosgraph

class WatcherThread(threading.Thread):

    def __init__(self,gateway_sync):
        # init thread
        threading.Thread.__init__(self)
        self.gateway_sync = gateway_sync
        self.master = gateway_sync.master
        self.cv = self.master.cv
        self.pubs = self.master.pubs_node
        self.public_interface = gateway_sync.public_interface
    
        # dumped interface is a mapping of whitelist regex to actual topics as they
        # become available
        self.dumped_interface = dict()
        self.dumped_interface['topic'] = set()
        self.dumped_interface['service'] = set()
        self.start()

    def run(self):
        print "Thread Started"
        while not rospy.is_shutdown():
            self.cv.acquire()

            # self.checkRemoteList()
            if self.gateway_sync.is_connected: 
                # 1. Check all remove interfaces are still in redis server, if it is gone, unregister it 
                #self.pollServer()
                # 2. Check all local public interfaces are still valid
                self.checkPublicInterfaces()
            self.cv.release()
            rospy.sleep(3.0)

    def pollServer(self):
        remotelist = self.gateway_sync.getRemoteLists()

        for master in remotelist:
            print str(remotelist[master])

    def checkPublicInterfaces(self):
        pubs, _, srvs = self.master.getSystemState() 
        self.update("topic",pubs)
        self.update("service",srvs)

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
                self.gateway_sync.unadvertise([string])

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
        #             self.gateway_sync.removePublicInterfaceByName(identifier, name)
        #             self.dumped_interface[identifier].remove(name)
  
        # add/remove named interfaces to flipped list as necessary
        for x in list:
            name = x[0]
            clients, non_clients = self.gateway_sync.getFlippedClientList(identifier, name)
            self.gateway_sync.addFlippedInterfaceByName(identifier,clients,name)
            self.gateway_sync.removeFlippedInterfaceByName(identifier,non_clients,name)

"""
  polling thread should do...
  1. unregister the unavailable remote topics/services
  2. remove the unavailable topics/services from public list
"""
