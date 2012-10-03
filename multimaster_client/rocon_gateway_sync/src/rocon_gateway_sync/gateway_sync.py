#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway_sync/LICENSE 
#

##############################################################################
# Imports
##############################################################################

import socket
import time
import re
import itertools
import json

import roslib; roslib.load_manifest('rocon_gateway_sync')
import rospy
import rosgraph
from std_msgs.msg import Empty

# Local imports
import utils
from .hub_api import Hub
from .master_api import LocalMaster
from .watcher_thread import WatcherThread
from .exceptions import GatewayError, ConnectionTypeError

##############################################################################
# Gateway
##############################################################################

'''
    The roles of GatewaySync is below
    1. communicate with ros master using xml rpc node
    2. communicate with redis server
'''

class GatewaySync(object):
    '''
    The gateway between ros system and redis server
    '''

    def __init__(self, name):
        self.unresolved_name = name # This gets used to build unique names after connection to the hub
        self.unique_name = None # single string value set after hub connection (note: it is not a redis rocon:: rooted key!)
        self.master_uri = None
        self.is_connected = False

        self.hub = Hub(self.processUpdate, self.unresolved_name)
        self.master = LocalMaster()
        self.master_uri = self.master.getMasterUri()

        # create a thread to clean-up unavailable topics
        self.watcher_thread = WatcherThread(self)

        # create a whitelist of named topics and services for public
        self.public_topic_whitelist = list()
        self.public_topic_blacklist = list()
        self.public_service_whitelist = list()
        self.public_service_blacklist = list()

        # create a whitelist/blacklist of named topics and services for flipped
        self.flipped_topic_whitelist = dict()
        self.flipped_service_whitelist = dict()
        self.flip_public_topics = set()

        #create a list of flipped triples
        self.flipped_interface_list = dict()

    def connectToHub(self,ip,port):
        try:
            self.hub.connect(ip,port)
            self.unique_name = self.hub.registerGateway()
            print "Obtained unique name: " + self.unique_name
            self.is_connected = True
        except Exception as e:
            print str(e)
            return False
        return True

    ##########################################################################
    # Public Interface Methods
    ##########################################################################
    
    def advertise(self,list):
        '''
        Adds a connection (topic/service/action) to the public
        interface.
        
        - adds to the ros manager so it can watch for changes
        - adds to the hub so it can be pulled by remote gateways
        
        @param list : list of connection representations (usually triples)
        '''
        if not self.is_connected:
            rospy.logerr("Gateway : advertise call failed [no hub connection].")
            return False, []
        try:
            for l in list:
                if self.master.addPublicInterface(l): # watching may repeatedly try and add, but return false if already present (not an error)
                    self.hub.advertise(l) # can raise InvalidConnectionTypeError exceptions
                    rospy.loginfo("Gateway : added connection to the public interface [%s]"%l)
        except ConnectionTypeError as e: 
            rospy.logerr("Gateway : %s"%str(e))
            return False, []
        return True, []
        
    def unadvertise(self,list):
        '''
        Removes a connection (topic/service/action) from the public
        interface.
        
        @param list : list of connection representations (usually stringified triples)
        '''
        if not self.is_connected:
            rospy.logerr("Gateway : unadvertise call failed [no hub connection].")
            return False, []
        try:
            for l in list:
                if self.master.removePublicInterface(l):
                    self.hub.unadvertise(l)
                    rospy.loginfo("Gateway : removed connection from the public interface [%s]"%l)
        except ConnectionTypeError as e: 
            rospy.logerr("Gateway : %s"%str(e))
            return False, []
        
        # inform other gateways of the change
        self.hub.broadcastTopicUpdate(json.dumps(['update','removing']))
        return True, []
       
    def addPublicTopicByName(self,topic):
        list = self.getTopicString([topic])
        return self.advertise(list)

    def addNamedTopics(self, list):
        print "Adding named topics: " + str(list)
        self.public_topic_whitelist.extend(list)
        return True, []

    def getTopicString(self,list):
        l = []
        for topic in list:
            try:
                topicinfo = self.master.getTopicInfo(topic)
            
                # there may exist multiple publisher
                for info in topicinfo:
                    l.append(topic+","+info)
            except:
                print "Error while looking up topic. Perhaps topic does not exist"
        return l

    def removePublicTopicByName(self,topic):
        # remove topics that exist, but are no longer part of the public interface
        list = self.getTopicString([topic])
        return self.unadvertise(list)

    def removeNamedTopics(self, list):
        print "Removing named topics: " + str(list)
        self.public_topic_whitelist[:] = [x for x in self.public_topic_whitelist if x not in list]
        return True, []

    def addPublicServiceByName(self,service):
        list = self.getServiceString([service])
        return self.advertise(list)

    def addNamedServices(self, list):
        print "Adding named services: " + str(list)
        self.public_service_whitelist.extend(list)
        return True, []

    def getServiceString(self,list):
        list_with_node_ip = []
        for service in list:
            #print service
            try:
                srvinfo = self.master.getServiceInfo(service)
                list_with_node_ip.append(service+","+srvinfo)
            except:
                print "Error obtaining service info. Perhaps service does not exist?"
        return list_with_node_ip


    def removePublicServiceByName(self,service):
        # remove available services that should no longer be on the public interface
        list = self.getServiceString([service])
        return self.unadvertise(list)

    def removeNamedServices(self, list):
        print "Removing named services: " + str(list)
        self.public_service_whitelist[:] = [x for x in self.public_service_whitelist if x not in list]
        return True, []

    def addPublicInterfaceByName(self, identifier, name):
        if identifier == "topic":
            self.addPublicTopicByName(name)
        elif identifier == "service":
            self.addPublicServiceByName(name)

    def removePublicInterfaceByName(self,identifier,name):
        if identifier == "topic":
            self.removePublicTopicByName(name)
        elif identifier == "service":
            self.removePublicServiceByName(name)

    def requestForeignTopic(self,list): 

        try:
            for line in list:
                topic, topictype, node_xmlrpc_uri = line.split(",")
                topic = utils.reshapeTopic(topic)
                node_xmlrpc_uri = utils.reshapeUri(node_xmlrpc_uri)
                if self.master.registerTopic(topic,topictype,node_xmlrpc_uri):
                    print "Adding foreign topic: " + line
        except Exception as e:
            print "In requestForeignTopic"
            raise
        
        return True, []

    def requestForeignService(self,list): 
        try:
            for line in list:
                service, service_api, node_xmlrpc_uri = line.split(",")
                service = utils.reshapeTopic(service)
                service_api = utils.reshapeUri(service_api)
                node_xmlrpc_uri = utils.reshapeUri(node_xmlrpc_uri)
                if self.master.registerService(service,service_api,node_xmlrpc_uri):
                    print "Adding foreign service: " + line
        except Exception as e:
            print "In requestForeignService"
            raise
        
        return True, []

    def unregisterForeignTopic(self,list):
        try:
            for line in list:
                print line
                topic, topictype, node_xmlrpc_uri = line.split(",")
                topic = utils.reshapeTopic(topic)
                node_xmlrpc_uri = utils.reshapeUri(node_xmlrpc_uri)
                if self.master.unregisterTopic(topic,topictype,node_xmlrpc_uri):
                    print "Removing foreign topic: " + line
        except Exception as e:
            print "In unregisterForeignTopic"
            raise
            
        return True, []

    def unregisterForeignService(self,list):
        try:
            for line in list:
                service, service_api, node_xmlrpc_uri = line.split(",")
                service = utils.reshapeTopic(service)
                service_api = utils.reshapeUri(service_api)
                node_xmlrpc_uri = utils.reshapeUri(node_xmlrpc_uri)
                if self.master.unregisterService(service,service_api,node_xmlrpc_uri):
                    print "Removing foreign service: " + line
        except Exception as e:
            print "In Unregister Foreign Service"
            raise
        
        return True, []

    def flipoutTopic(self,list):
        # list[0] # of channel
        # list[1:list[0]] is channels
        # rest of them are fliping topics
        try:
            num = int(list[0])
            channels = list[1:num+1]
            topics = list[num+1:len(list)]
            if num == 0 or len(topics) == 0:
                return False, []

            for chn in channels:
                print "Flipping out topics: " + str(topics) + " to " + chn
                self.hub.flip("flipouttopic",chn,topics)
#                self.flipout("flipouttopic",chn,topics)
        except:
            return False, []

        return True, []

    def addNamedFlippedTopics(self, list):
        # list[0] # of channel
        # list[1:list[0]] is channels
        # rest of them are fliping topics
        num = int(list[0])
        channels = list[1:num+1]
        topics = list[num+1:len(list)]
        print "Adding named topics to flip: " + str(list)
        for chn in channels:
            if chn not in self.flipped_topic_whitelist:
                self.flipped_topic_whitelist[chn] = set()
            self.flipped_topic_whitelist[chn].update(set(topics))
        return True, []

    def addFlippedTopicByName(self,clients,name):
        topic_triples = self.getTopicString([name])
        for client in clients:
            if client not in self.flipped_interface_list:
                self.flipped_interface_list[client] = set()
            add_topic_triples = [x for x in topic_triples if x not in self.flipped_interface_list[client]]
            self.flipped_interface_list[client].update(set(add_topic_triples))
            topic_list = list(itertools.chain.from_iterable([[1, client], add_topic_triples]))
            self.flipoutTopic(topic_list)

    def removeFlippedTopic(self,list):
        # list[0] # of channel
        # list[1:list[0]] is channels
        # rest of them are fliping topics
        try:
            num = int(list[0])
            channels = list[1:num+1]
            topics = list[num+1:len(list)]
            if num == 0 or len(topics) == 0:
                return False, []

            for chn in channels:
                print "Removing fipped out topics: " + str(topics) + " to " + chn
                self.flipout("removeflippedtopic",chn,topics)
        except:
            return False, []

        return True, []

    def removeFlippedTopicByName(self,clients,name):
        topic_triples = self.getTopicString([name])
        for client in clients:
            if client not in self.flipped_interface_list:
                continue
            delete_topic_triples = [x for x in topic_triples if x in self.flipped_interface_list[client]]
            self.flipped_interface_list[client].difference_update(set(delete_topic_triples))
            topic_list = list(itertools.chain.from_iterable([[1, client], delete_topic_triples]))
            self.removeFlippedTopic(topic_list)

    def removeNamedFlippedTopics(self,list):
        # list[0] # of channel
        # list[1:list[0]] is channels
        # rest of them are fliping topics
        num = int(list[0])
        channels = list[1:num+1]
        topics = list[num+1:len(list)]
        print "removing named topics from flip: " + str(list)
        for chn in channels:
            if chn in self.flipped_topic_whitelist:
                self.flipped_topic_whitelist[chn].difference_update(set(topics))
        return True, []

    def flipoutService(self,list):
        # list[0] # of channel
        # list[1:list[0]] is channels
        # rest of them are fliping services
        try:
            num = int(list[0])
            channels = list[1:num+1]
            services = list[num+1:len(list)]
            if num == 0 or len(services) == 0:
                return False, []

            for chn in channels:
                print "Flipping out services: " + str(services) + " to " + chn
                self.flipout("flipoutservice",chn,services)
        except Exception as e:
            print str(e)
            return False, []
        return True, []

    def addFlippedServiceByName(self,clients,name):
        service_triples = self.getServiceString([name])
        for client in clients:
            if client not in self.flipped_interface_list:
                self.flipped_interface_list[client] = set()
            add_service_triples = [x for x in service_triples if x not in self.flipped_interface_list[client]]
            self.flipped_interface_list[client].update(set(add_service_triples))
            service_list = list(itertools.chain.from_iterable([[1, client], add_service_triples]))
            self.flipoutService(service_list)

    def addNamedFlippedServices(self, list):
        # list[0] # of channel
        # list[1:list[0]] is channels
        # rest of them are fliping services
        num = int(list[0])
        channels = list[1:num+1]
        services = list[num+1:len(list)]
        print "Adding named services to flip: " + str(list)
        for chn in channels:
            if chn not in self.flipped_service_whitelist:
                self.flipped_service_whitelist[chn] = set()
            self.flipped_service_whitelist[chn].update(set(services))
        return True, []

    def removeFlippedService(self,list):
        # list[0] # of channel
        # list[1:list[0]] is channels
        # rest of them are fliping services
        try:
            num = int(list[0])
            channels = list[1:num+1]
            services = list[num+1:len(list)]
            if num == 0 or len(services) == 0:
                return False, []

            for chn in channels:
                print "Removing flipped out services: " + str(services) + " to " + chn
                self.flipout("removeflippedservice",chn,services)
        except Exception as e:
            print str(e)
            return False, []
        return True, []

    def removeFlippedServiceByName(self,clients,name):
        service_triples = self.getServiceString([name])
        for client in clients:
            if client not in self.flipped_interface_list:
                continue
            delete_service_triples = [x for x in service_triples if x in self.flipped_interface_list[client]]
            self.flipped_interface_list[client].difference_update(set(delete_service_triples))
            service_list = list(itertools.chain.from_iterable([[1, client], delete_service_triples]))
            self.removeFlippedService(service_list)

    def removeNamedFlippedServices(self,list):
        # list[0] # of channel
        # list[1:list[0]] is channels
        # rest of them are fliping services
        num = int(list[0])
        channels = list[1:num+1]
        services = list[num+1:len(list)]
        print "removing named services from flip: " + str(list)
        for chn in channels:
            if chn in self.flipped_service_whitelist:
                self.flipped_service_whitelist[chn].difference_update(set(services))
        return True, []

    def addFlippedInterfaceByName(self,identifier,clients,name):
        if identifier == 'topic':
            self.addFlippedTopicByName(clients,name)
        elif identifier == 'service':
            self.addFlippedServiceByName(clients,name)

    def removeFlippedInterfaceByName(self,identifier,clients,name):
        if identifier == 'topic':
            self.removeFlippedTopicByName(clients,name)
        elif identifier == 'service':
            self.removeFlippedServiceByName(clients,name)

    def flipAll(self,list):
        #list is channels
        for chn in list:
            if chn not in self.flipped_topic_whitelist:
              self.flipped_topic_whitelist[chn] = set()
            if chn not in self.flipped_service_whitelist:
              self.flipped_service_whitelist[chn] = set()
            self.flipped_topic_whitelist[chn].add('.*')
            self.flipped_service_whitelist[chn].add('.*')
            if chn in self.flip_public_topics:
                self.flip_public_topics.remove(chn)
        return True, []

    def flipAllPublic(self,list):
        #list is channels
        for chn in list:
            if chn in self.flipped_topic_whitelist:
              self.flipped_topic_whitelist[chn].difference_update(set(['.*']))
            if chn in self.flipped_service_whitelist:
              self.flipped_service_whitelist[chn].difference_update(set(['.*']))
            self.flip_public_topics.add(chn)
        return True, []

    def flipListOnly(self,list):
        #list is channels
        for chn in list:
            if chn in self.flipped_topic_whitelist:
              self.flipped_topic_whitelist[chn].difference_update(set(['.*']))
            if chn in self.flipped_service_whitelist:
              self.flipped_service_whitelist[chn].difference_update(set(['.*']))
            if chn in self.flip_public_topics:
                self.flip_public_topics.remove(chn)
        return True, []

    def makeAllPublic(self,list):
        print "Dumping all non-blacklisted interfaces"
        self.public_topic_whitelist.append('.*')
        self.public_service_whitelist.append('.*')
        return True, []

    def removeAllPublic(self,list):
        print "Resuming dump of explicitly whitelisted interfaces"
        self.public_topic_whitelist[:] = [x for x in self.public_topic_whitelist if x != '.*']
        self.public_service_whitelist[:] = [x for x in self.public_service_whitelist if x != '.*']
        return True, []

    def allowInterface(self,name,whitelist,blacklist):
        in_whitelist = False
        in_blacklist = False
        for x in whitelist:
            if re.match(x, name):
                in_whitelist = True
                break
        for x in blacklist:
            if re.match(x, name):
                in_blacklist = True
                break

        return in_whitelist and (not in_blacklist)

    def allowInterfaceInPublic(self,identifier,name):
        if identifier == 'topic':
            whitelist = self.public_topic_whitelist
            blacklist = self.public_topic_blacklist
        else:
            whitelist = self.public_service_whitelist
            blacklist = self.public_service_blacklist
        return self.allowInterface(name,whitelist,blacklist)

    def allowInterfaceInFlipped(self,identifier,client,name):
        #print '  testing ' + identifier + ': ' + name + ' for ' + client
        if client in self.flip_public_topics:
          #print '    client in public list'
          return self.allowInterfaceInPublic(identifier,name)

        if identifier == 'topic':
            if client not in self.flipped_topic_whitelist:
                return False
            whitelist = self.flipped_topic_whitelist[client]
            blacklist = self.public_topic_blacklist
        else:
            if client not in self.flipped_service_whitelist:
                return False
            whitelist = self.flipped_service_whitelist[client]
            blacklist = self.public_service_blacklist
        return self.allowInterface(name,whitelist,blacklist)

    def getFlippedClientList(self,identifier,name):
        list = self.hub.listPublicInterfaces()
        allowed_clients = []
        not_allowed_clients = []
        for chn in list:
            if self.allowInterfaceInFlipped(identifier,chn,name):
                allowed_clients.append(chn)
            else:
                not_allowed_clients.append(chn)
        return [allowed_clients, not_allowed_clients]

    def clearServer(self):
        self.hub.unregisterGateway()
        self.master.clear()

    def processUpdate(self,msg):
        '''
          Used as a callback for incoming requests on redis pubsub channels.
          It gets assigned to RedisManager.callback.
        '''

        try:
            msg = json.loads(msg)
            cmd = msg[0]
            provider = msg[1]
            rest = msg[2:len(msg)]

            if cmd == "flipouttopic":
                self.requestForeignTopic(rest)
            elif cmd == "flipoutservice":
                self.requestForeignService(rest)
            elif cmd == "removeflippedtopic":
                self.unregisterForeignTopic(rest)
            elif cmd == "removeflippedservice":
                self.unregisterForeignService(rest)
            elif cmd == "update":
                # print "HERE"
                # print str(rest)
                pass
            else:
                print "error"
        except:
            print "Wrong Message : " + str(msg)

    def getInfo(self):
        return self.unique_name
