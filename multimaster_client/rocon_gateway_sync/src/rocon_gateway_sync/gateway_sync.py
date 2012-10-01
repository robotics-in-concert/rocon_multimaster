#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Yujin Robot, Daniel Stonier, Jihoon Lee
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#    * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above
#        copyright notice, this list of conditions and the following
#        disclaimer in the documentation and/or other materials provided
#        with the distribution.
#    * Neither the name of Yujin Robot nor the names of its
#        contributors may be used to endorse or promote products derived
#        from this software without specific prior written permission.
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
import re
import itertools
import json

import roslib; roslib.load_manifest('rocon_gateway_sync')
import rospy
import rosgraph
from std_msgs.msg import Empty

from cleanup_thread import CleanupThread
from .redis_manager import RedisManager
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

    masterlist = 'rocon:masterlist'
    master_uri = None
    update_topic = 'rocon:update'
    index = 'rocon:hub:index'
    unique_name = None
    connected = False

    def __init__(self):
        self.redis_manager = RedisManager(self.processUpdate)
        self.ros_manager = ROSManager()
        self.master_uri = self.ros_manager.getMasterUri()

        # create a thread to clean-up unavailable topics
        self.cleanup_thread = CleanupThread(self)

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

    def connectToRedisServer(self,ip,port):
        try:
            self.redis_manager.connect(ip,port)
            self.unique_name = self.redis_manager.registerClient(self.masterlist,self.index,self.update_topic)
            print "Obtained unique name: " + self.unique_name
            self.connected = True
        except Exception as e:
            print str(e)
            return False
        return True

    def getRemoteLists(self):
        remotelist = {}
        masterlist = self.redis_manager.getMembers(self.masterlist)

        for master in masterlist:
            remotelist[master] = {}
            
            # get public topic list of this master
            key = master +":topic"
            remotelist[master]['topic'] = self.redis_manager.getMembers(key)

            # get public service list of this master
            key = master +":service"
            remotelist[master]['service'] = self.redis_manager.getMembers(key)

        return remotelist        

    def addPublicTopics(self,list):
        if not self.connected:
            print "It is not connected to Server"
            return False, []
        key = self.unique_name + ":topic"

        # figures out each topics node xmlrpc_uri and attach it on topic
        try:
            for l in list:
                if self.ros_manager.addPublicInterface("topic",l):
                    print "Adding topic : " + str(l)
                    self.redis_manager.addMembers(key,l)

        except Exception as e:
            print str(e)
            return False, []

        return True, []

    def addPublicTopicByName(self,topic):
        list = self.getTopicString([topic])
        return self.addPublicTopics(list)

    def addNamedTopics(self, list):
        print "Adding named topics: " + str(list)
        self.public_topic_whitelist.extend(list)
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
        if not self.connected:
            print "It is not connected to Server"
            return False, []

        '''
            this also stop publishing topic to remote server
        '''
#self.redis_manager.removeMembers(key,list)
        key = self.unique_name + ":topic"
        for l in list:
            if self.ros_manager.removePublicInterface("topic",l):
                print "Removing topic : " + l
                self.redis_manager.removeMembers(key,l)

        #self.redis_manager.sendMessage(self.update_topic,"update-removing")
        return True, []

    def removePublicTopicByName(self,topic):
        # remove topics that exist, but are no longer part of the public interface
        list = self.getTopicString([topic])
        return self.removePublicTopics(list)

    def removeNamedTopics(self, list):
        print "Removing named topics: " + str(list)
        self.public_topic_whitelist[:] = [x for x in self.public_topic_whitelist if x not in list]
        return True, []

    def addPublicService(self,list):
        if not self.connected:
            print "It is not connected to Server"
            return False, []

        key = self.unique_name + ":service"
        try:
            for l in list:
                if self.ros_manager.addPublicInterface("service",l):
                    print "Adding Service : " + str(l)
                    self.redis_manager.addMembers(key,l)
        except Exception as e:
            print str(e)
            return False, []

        return True, []

    def addPublicServiceByName(self,service):
        list = self.getServiceString([service])
        return self.addPublicService(list)

    def addNamedServices(self, list):
        print "Adding named services: " + str(list)
        self.public_service_whitelist.extend(list)
        return True, []

    def getServiceString(self,list):
        list_with_node_ip = []
        for service in list:
            #print service
            srvinfo = self.ros_manager.getServiceInfo(service)
            list_with_node_ip.append(service+","+srvinfo)
        return list_with_node_ip


    def removePublicService(self,list):
        if not self.connected:
            print "It is not connected to Server"
            return False, []

        key = self.unique_name + ":service"
        for l in list:
            if self.ros_manager.removePublicInterface("service",l):
                print "Removing service : " + l
                self.redis_manager.removeMembers(key,l)

        return True, []

    def removePublicServiceByName(self,service):
        # remove available services that should no longer be on the public interface
        list = self.getServiceString([service])
        return self.removePublicService(list)

    def removeNamedServices(self, list):
        print "Removing named services: " + str(list)
        self.public_service_whitelist[:] = [x for x in self.public_service_whitelist if x not in list]
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
                if self.ros_manager.registerTopic(topic,topictype,node_xmlrpc_uri):
                    print "Adding foreign topic: " + line
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
                if self.ros_manager.registerService(service,service_api,node_xmlrpc_uri):
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
                topic = self.reshapeTopic(topic)
                node_xmlrpc_uri = self.reshapeUri(node_xmlrpc_uri)
                if self.ros_manager.unregisterTopic(topic,topictype,node_xmlrpc_uri):
                    print "Removing foreign topic: " + line
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
                if self.ros_manager.unregisterService(service,service_api,node_xmlrpc_uri):
                    print "Removing foreign service: " + line
        except Exception as e:
            print "In Unregister Foreign Service"
            raise
        
        return True, []

    def flipout(self,cmd,channel,list):
        cmd = json.dumps([cmd,self.unique_name] + list)

        try:
            self.redis_manager.sendMessage(channel,cmd)
        except Exception as e:
            return False

        return True

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
                self.flipout("flipouttopic",chn,topics)
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
              self.flipped_topic_whitelist[chn].remove('.*')
            if chn not in self.flipped_service_whitelist:
              self.flipped_service_whitelist[chn].remove('.*')
            self.flip_public_topics.add(chn)
        return True, []

    def flipListOnly(self,list):
        #list is channels
        for chn in list:
            if chn in self.flipped_topic_whitelist:
              self.flipped_topic_whitelist[chn].remove('.*')
            if chn not in self.flipped_service_whitelist:
              self.flipped_service_whitelist[chn].remove('.*')
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
        if client in self.flip_public_topics:
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
        if identifier == 'topic':
            list = self.flipped_topic_whitelist
        else:
            list = self.flipped_service_whitelist

        return [chn for chn in list if self.allowInterfaceInFlipped(identifier,chn,name)], [chn for chn in list if not self.allowInterfaceInFlipped(identifier,chn,name)]

    def reshapeUri(self,uri):
        if uri[len(uri)-1] is not '/':
            uri = uri + '/'
        return uri

    def reshapeTopic(self,t):
        if t[0] is not '/':
            t = '/' + t
        return t

    def clearServer(self):
        self.redis_manager.unregisterClient(self.masterlist,self.unique_name)
        self.ros_manager.clear()

    def processUpdate(self,msg):

        try:
            msg = json.loads(msg)
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


    def validateWhiteList(self,provider):
        # There is no validation method yet
#print str(provider)

        return True

    def post(self,msg):
        command, key, member = msg 

#print "Posting : " + str(msg)
        try:
            if command == "addmember":
                self.redis_manager.addMembers(key,member)
            elif command == "removemember":
                self.redis_manager.removeMembers(key,member)
            elif command == "getmembers":
                member_list = self.redis_manager.getMembers(key)
                return True, member_list
            else:
                print "Error Wrong command %s",command
        except Exception as e:
            print str(e)
            return False, []

        return True, []

    def getInfo(self):
        return self.unique_name
