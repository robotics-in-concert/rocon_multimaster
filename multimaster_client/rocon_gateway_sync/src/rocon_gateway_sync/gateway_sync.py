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

    masterlist = 'masterlist'
    master_uri = None
    update_topic = 'update'
    index = 'index'
    unique_name = None
    connected = False

    def __init__(self):
        self.redis_manager = RedisManager(self.processUpdate)
        self.ros_manager = ROSManager()

        # create a thread to clean-up unavailable topics
        self.cleanup_thread = CleanupThread(self)


    def connectToRedisServer(self,ip,port):
        try:
            # 1. connect to redis server
            self.redis_manager.connect(ip,port)

            self.master_uri = self.ros_manager.getMasterUri()
            self.unique_name = self.redis_manager.registerClient(self.masterlist,self.index,self.update_topic)

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
            return False
        key = self.unique_name + ":topic"

        # figures out each topics node xmlrpc_uri and attach it on topic
        try:
            list_with_node_ip = self.getTopicString(list)

            for l in list_with_node_ip:
                print "Adding topic : " + str(l)
                if self.ros_manager.addPublicInterface("topic",l):
                    self.redis_manager.addMembers(key,l)

        except Exception as e:
            print str(e)
            return False

        return True

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
            return False

        '''
            this also stop publishing topic to remote server
        '''
#self.redis_manager.removeMembers(key,list)
        for l in list:
            self.removePublicInterface("topic",l)

        self.redis_manager.sendMessage(self.update_topic,"update-removing")
        return True


    def addPublicService(self,list):
        if not self.connected:
            print "It is not connected to Server"
            return False

        key = self.unique_name + ":service"
        try:
            list_with_node_ip = self.getServiceString(list)

            for l in list_with_node_ip:
                print "Adding Service : " + str(l)
                self.ros_manager.addPublicInterface("service",l)
                self.redis_manager.addMembers(key,l)
        except Exception as e:
            print str(e)
            return False

        return True

    def removePublicInterface(self,identifier,string):
        print "Removing "+ identifier + " : " + string
        self.ros_manager.removePublicInterface(identifier,string)
        key = self.unique_name + ":"+identifier
        self.redis_manager.removeMembers(key,string)

    def getServiceString(self,list):
        list_with_node_ip = []
        for service in list:
            srvinfo = self.ros_manager.getServiceInfo(service)
            list_with_node_ip.append(service+","+srvinfo)
        return list_with_node_ip


    def removePublicService(self,list):
        if not self.connected:
            print "It is not connected to Server"
            return False

        for l in list:
            self.removePublicInterface("service",l)

        return True


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
        
        return True

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
        
        return True

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
            
        return True

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
        
        return True


    def makeAllPublic(self,msg):
        pub, sub, srv = self.ros_manager.getSystemState()

        return True


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
                print str(rest)
            else:
                print "error"
        except:
            print "Wrong Message : " + str(msg)

    def flipout(self,cmd,channel,list):
        cmd = cmd + "-" + self.unique_name
        for tinfo in list:
            cmd = cmd + "-" + tinfo

        try:
            self.redis_manager.sendMessage(channel,cmd)
        except Exception as e:
            return False

        return True

    def validateWhiteList(self,provider):
        # There is no validation method yet
        print str(provider)

        return True

    def post(self,msg):
        command, key, member = msg 

        print "Posting : " + str(msg)
        try:
            if command == "addmember":
                self.redis_manager.addMembers(key,member)
            elif command == "removemember":
                self.redis_manager.removeMembers(key,member)
            else:
                print "Error Wrong command %s",command
        except Exception as e:
            print str(e)
            return False

        return True



