#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway_sync/LICENSE 
#

import socket
import time
import re
import roslib; roslib.load_manifest('rocon_gateway_sync')
import rospy
import rosgraph
from std_msgs.msg import Empty

from watcher_thread import WatcherThread
from .hub import Hub
from .ros_manager import ROSManager

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
        self.unique_name = None
        self.master_uri = None
        self.is_connected = False

        self.hub = Hub(self.processUpdate, self.unresolved_name)
        self.ros_manager = ROSManager()
        self.master_uri = self.ros_manager.getMasterUri()

        # create a thread to clean-up unavailable topics
        self.watcher_thread = WatcherThread(self)

        # create a whitelist of named topics and services
        self.topic_whitelist = list()
        self.topic_blacklist = list()
        self.service_whitelist = list()
        self.service_blacklist = list()

    def connectToHub(self,ip,port):
        try:
            self.hub.connect(ip,port)
            self.unique_name = self.hub.registerGateway()
            self.is_connected = True
        except Exception as e:
            print str(e)
            return False
        return True

    def addPublicTopics(self,list):
        '''
        Adds a topic triple to the public interface.
        
        - adds to the ros manager so it can watch for changes
        - adds to the hub so it can be pulled by remote gateways
        
        @param list : list of topic triples
        '''
        if not self.is_connected:
            rospy.logerr("Gateway : not connected to a hub.")
            return False, []

        # figures out each topics node xmlrpc_uri and attach it on topic
        try:
            for l in list:
                if self.ros_manager.addPublicInterface("topic",l):
                    print "Adding topic : " + str(l)
                    self.hub.advertise(l)

        except Exception as e:
            print str(e)
            return False, []

        return True, []

    def addPublicTopicByName(self,topic):
        list = self.getTopicString([topic])
        return self.addPublicTopics(list)

    def addNamedTopics(self, list):
        print "Adding named topics: " + str(list)
        self.topic_whitelist.extend(list)
        return True, []

    def getTopicString(self,list):
        l = [] 
        for topic in list:
            topicinfo = self.ros_manager.getTopicInfo(topic)
            
            # there may exist multiple publisher
            for info in topicinfo:
                l.append(topic+","+info)
        return l

    def removePublicTopics(self,list):
        if not self.is_connected:
            print "It is not connected to Server"
            return False, []

        '''
            this also stop publishing topic to remote server
        '''
#self.hub.removeMembers(key,list)
        key = self.unique_name + ":topic"
        for l in list:
            if self.ros_manager.removePublicInterface("topic",l):
                print "Removing topic : " + l
                self.hub.removeMembers(key,l)

        self.hub.broadcastTopicUpdate("update-removing")
        return True, []

    def removePublicTopicByName(self,topic):
        # remove topics that exist, but are no longer part of the public interface
        list = self.getTopicString([topic])
        return self.removePublicTopics(list)

    def removeNamedTopics(self, list):
        print "Removing named topics: " + str(list)
        self.topic_whitelist[:] = [x for x in self.topic_whitelist if x not in list]
        return True, []

    def addPublicService(self,list):
        if not self.is_connected:
            rospy.logwarn("Gateway : cannot add services, gateway is not connected to the hub.")
            return False, []

        try:
            for l in list:
                if self.ros_manager.addPublicInterface("service",l):
                    print "Adding Service : " + str(l)
                    self.hub.advertise(l)
        except Exception as e:
            print str(e)
            return False, []

        return True, []

    def addPublicServiceByName(self,service):
        list = self.getServiceString([service])
        return self.addPublicService(list)

    def addNamedServices(self, list):
        print "Adding named services: " + str(list)
        self.service_whitelist.extend(list)
        return True, []

    def getServiceString(self,list):
        list_with_node_ip = []
        for service in list:
            #print service
            srvinfo = self.ros_manager.getServiceInfo(service)
            list_with_node_ip.append(service+","+srvinfo)
        return list_with_node_ip


    def removePublicService(self,list):
        if not self.is_connected:
            print "It is not connected to Server"
            return False, []

        key = self.unique_name + ":service"
        for l in list:
            if self.ros_manager.removePublicInterface("service",l):
                print "Removing service : " + l
                self.hub.removeMembers(key,l)

        return True, []

    def removePublicServiceByName(self,service):
        # remove available services that should no longer be on the public interface
        list = self.getServiceString([service])
        return self.removePublicService(list)

    def removeNamedServices(self, list):
        print "Removing named services: " + str(list)
        self.service_whitelist[:] = [x for x in self.service_whitelist if x not in list]
        return True, []

    def addPublicInterfaceByName(self, identifier, name):
        print "apin"
        if identifier == "topic":
            self.addPublicTopicByName(name)
        elif identifier == "service":
            self.addPublicServiceByName(name)

    def removePublicInterface(self,identifier,string):
        if identifier == "topic":
            self.removePublicTopics([string])
        elif identifier == "service":
            self.removePublicService([string])

    def removePublicInterfaceByName(self,identifier,name):
        print "rpin"
        if identifier == "topic":
            self.removePublicTopicByName(name)
        elif identifier == "service":
            self.removePublicServiceByName(name)

    def requestForeignTopic(self,list): 

        try:
            for line in list:
                topic, topictype, node_xmlrpc_uri = line.split(",")
                topic = self.reshapeTopic(topic)
                node_xmlrpc_uri = self.reshapeUri(node_xmlrpc_uri)
                print "Adding : " + line
                self.ros_manager.registerTopic(topic,topictype,node_xmlrpc_uri)
        except Exception as e:
            print "In requestForeignTopic"
            raise
        
        return True, []

    def requestForeignService(self,list): 
        try:
            for line in list:
                service, service_api, node_xmlrpc_uri = line.split(",")
                service = self.reshapeTopic(service)
                service_api = self.reshapeUri(service_api)
                node_xmlrpc_uri = self.reshapeUri(node_xmlrpc_uri)
                print "Adding : " + line
                self.ros_manager.registerService(service,service_api,node_xmlrpc_uri)
        except Exception as e:
            print "In requestForeignService"
            raise
        
        return True, []

    def unregisterForeignTopic(self,list):
        try:
            for line in list:
                print line
                topic, topictype, node_xmlrpc_uri = line.split(",")
                topic = self.reshapeTopic(topic)
                node_xmlrpc_uri = self.reshapeUri(node_xmlrpc_uri)
                self.ros_manager.unregisterTopic(topic,topictype,node_xmlrpc_uri)                
        except Exception as e:
            print "In unregisterForeignTopic"
            raise
            
        return True, []

    def unregisterForeignService(self,list):
        try:
            for line in list:
                service, service_api, node_xmlrpc_uri = line.split(",")
                service = self.reshapeTopic(service)
                service_api = self.reshapeUri(service_api)
                node_xmlrpc_uri = self.reshapeUri(node_xmlrpc_uri)
                self.ros_manager.unregisterService(service,service_api,node_xmlrpc_uri)
        except Exception as e:
            print "In Unregister Foreign Service"
            raise
        
        return True, []


    def makeAllPublic(self,list):
        print "Dumping all non-blacklisted interfaces"
        self.topic_whitelist.append('.*')
        self.service_whitelist.append('.*')
        return True, []

    def removeAllPublic(self,list):
        print "Resuming dump of explicitly whitelisted interfaces"
        self.topic_whitelist[:] = [x for x in self.topic_whitelist if x != '.*']
        self.service_whitelist[:] = [x for x in self.service_whitelist if x != '.*']
        return True, []

    def allowInterfaceInDump(self,identifier,name):
        if identifier == 'topic':
            whitelist = self.topic_whitelist
            blacklist = self.topic_blacklist
        else:
            whitelist = self.service_whitelist
            blacklist = self.service_blacklist

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

    def reshapeUri(self,uri):
        if uri[len(uri)-1] is not '/':
            uri = uri + '/'
        return uri

    def reshapeTopic(self,t):
        if t[0] is not '/':
            t = '/' + t
        return t


    def clearServer(self):
        self.hub.unregisterGateway(self.unique_name)
        self.ros_manager.clear()

    def processUpdate(self,msg):
        '''
          Used as a callback for incoming requests on redis pubsub channels.
          It gets assigned to RedisManager.callback.
        '''

        try:
            msg = msg.split("-")
            cmd = msg[0]
            provider = msg[1]
            rest = msg[2:len(msg)]

            if not self.validateWhiteList(provider):
                print str(msg) + "couldn't pass the white list validation"
                return

            if cmd == "flipouttopic":
                self.requestForeignTopic(rest)
            elif cmd == "flipoutservice":
                self.requestForeignService(rest)
            elif cmd == "update":
                # print "HERE"
                # print str(rest)
                pass
            else:
                print "error"
        except:
            print "Wrong Message : " + str(msg)

    def flipout(self,cmd,channel,list):
        cmd = cmd + "-" + self.unique_name
        for tinfo in list:
            cmd = cmd + "-" + tinfo

        try:
            self.hub.sendMessage(channel,cmd)
        except Exception as e:
            return False

        return True

    def validateWhiteList(self,provider):
        # There is no validation method yet
#print str(provider)

        return True

    def post(self,msg):
        command, key, member = msg 

#print "Posting : " + str(msg)
        try:
            if command == "addmember":
                self.hub.addMembers(key,member)
            elif command == "removemember":
                self.hub.removeMembers(key,member)
            elif command == "getmembers":
                member_list = self.hub.getMembers(key)
                return True, member_list
            else:
                print "Error Wrong command %s",command
        except Exception as e:
            print str(e)
            return False, []

        return True, []

    def getInfo(self):
        return self.unique_name
