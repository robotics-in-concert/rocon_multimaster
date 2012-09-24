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

import roslib; roslib.load_manifest('rocon_gateway')
import rospy
import rosgraph
import rocon_gateway
from gateway_comms.msg import *
from gateway_comms.srv import *
from zeroconf_comms.srv import *
from rocon_gateway_sync import *



# This class is wrapper ros class of gateway sync.
# The role of this node is below
# 1. listens to server up/down status from zero configuration node
# 2. listens to local ros node's remote topic registration request
# 3. response a local ros node's get remote topic/service list request
class Gateway():

    # Zeroconfiguratin topics
    zeroconf_service = "_ros-gateway-hub._tcp"
    zeroconf_connection_service = "~list_discovered_services"

    # request from local node
    local_request_name = "~request"

    gateway_info_srv_name = "~info"
    gateway_info_srv = None

    gateway_sync = None
    param = {}
    is_connected = False
    allow_random_redis_server = False
    callbacks = {}

    def __init__(self):

        # Instantiate a GatewaySync module. This will take care of all redis server connection, communicatin with ros master uri
        self.gateway_sync = GatewaySync()

        self.setupCallbacks()
        self.parse_params()

        # Subscribe from zero conf new connection
        rospy.loginfo("gw: wait for zeroconf service...")
        rospy.wait_for_service(self.zeroconf_connection_service)
        self.zeroconf_service_proxy = rospy.ServiceProxy(self.zeroconf_connection_service,ListDiscoveredServices)
        rospy.loginfo("gw: done")

        # Service Server for local node requests
        self.remote_list_srv = rospy.Service(self.local_request_name,PublicHandler,self.processLocalRequest)

        # Service Server for gateway info
        self.gateway_info_srv = rospy.Service(self.gateway_info_srv_name,GatewayInfo,self.processGatewayInfo)

    def setupCallbacks(self):
        callbacks = self.callbacks
        callbacks["get_public_interfaces"] = self.processRemoteListRequest

        callbacks["add_public_topic"] = self.gateway_sync.addPublicTopics
        callbacks["remove_public_topic"] = self.gateway_sync.removePublicTopics 

        callbacks["add_public_service"] = self.gateway_sync.addPublicService
        callbacks["remove_public_service"] = self.gateway_sync.removePublicService

        callbacks["register_foreign_topic"] = self.gateway_sync.requestForeignTopic
        callbacks["unregister_foreign_topic"] = self.gateway_sync.unregisterForeignTopic

        callbacks["register_foreign_service"] = self.gateway_sync.requestForeignService
        callbacks["unregister_foreign_service"] = self.gateway_sync.unregisterForeignService

        callbacks["make_all_public"] = self.gateway_sync.makeAllPublic
     
        callbacks["flipout_topic"] = self.flipoutTopic
        callbacks["flipout_service"] = self.flipoutService

        callbacks["post"] = self.gateway_sync.post


    def parse_params(self):
        # Local topics and services to register redis server
        self.param['local_public_topic'] = rospy.get_param('~local_public_topic','')
        self.param['local_public_service'] = rospy.get_param('~local_public_service','')

        # Topics and services that need from remote server
#        self.param['remote_topic'] = rospy.get_param('~remote_topic','')
#        self.param['remote_service'] = rospy.get_param('~remote_service','')
        self.param['whitelist'] = rospy.get_param('~whitelist','')
        self.param['blacklist'] = rospy.get_param('~blacklist','')

        # if both whitelist and blacklist are empty, it connects to any discovered redis server
        if len(self.param['whitelist']) == 0:
            self.allow_random_redis_server = True

    def processLocalRequest(self,request):
        command = request.command
        success = False
        resp = PublicHandlerResponse()
        resp.success = success

        if command not in self.callbacks.keys():
            print "Wrong Command = " + str(command)
            return resp

        try:
            success, lists = self.callbacks[command](request.list)
        except Exception as e:
            print str(e)
            return resp

        if command == "get_public_interfaces":
            resp.remotelist = lists
            resp.success = success
            resp.concertmaster_list = []
        elif command == "post":
            resp.remotelist = []
            resp.success = success
            resp.concertmaster_list = lists
        else:
            resp.success = success

        return resp

    def processGatewayInfo(self,msg):
        return GatewayInfoResponse(self.gateway_sync.getInfo())
        

    # This function receives a service request from local ros node, crawl remote topic/service list from redis, and respose to local ros node.
    def processRemoteListRequest(self,msg):
        remote_list = self.gateway_sync.getRemoteLists()

        rl = []

        for host in remote_list.keys():
            l = RemoteList()
            l.hostname = host
            l.topics = remote_list[host]['topic']
            l.services= remote_list[host]['service']
            rl.append(l)
            
        return True, rl

    def flipoutTopic(self,list):
        # list[0] # of channel
        # list[1:list[0]] is channels
        # rest of them are fliping topics
        try:
            num = int(list[0])
            channels = list[1:num+1]
            topics = list[num+1:len(list)]
#topics = self.gateway_sync.getTopicString(topics)

            for chn in channels:
                print "Flipping out : " + str(topics) + " to " + chn
                self.gateway_sync.flipout("flipouttopic",chn,topics)
        except:
            return False, []

        return True, []

    def flipoutService(self,list):
        # list[0] # of channel
        # list[1:list[0]] is channels
        # rest of them are fliping services
        try:
            num = int(list[0])
            channels = list[1:num+1]
            services = list[num+1:len(list)]
#services = self.gateway_sync.getServiceString(services)

            for chn in channels:
                print "Flipping out : " + str(services) + " to " + chn
                self.gateway_sync.flipout("flipoutservice",chn,services)
        except Exception as e:
            print str(e)
            return False, []
        return True, []


    # It clears this client's information from redis-server
    def clearServer(self):
        try:
            self.gateway_sync.clearServer()
        except Exception as e:
            print str(e)

        print "Server cleared"


    def connect(self,msg):
        (ip, port) = rocon_gateway.resolveZeroconfAddress(msg)
        if self.gateway_sync.connectToRedisServer(ip,port):
            return True
        else:
            return False

    def spin(self):
        previously_found_hubs = []
        while not rospy.is_shutdown() and not self.is_connected:

            # Get discovered redis server list from zeroconf
            req = ListDiscoveredServicesRequest() 
            req.service_type = self.zeroconf_service
            resp = self.zeroconf_service_proxy(req)
            
            rospy.loginfo("gw: checking for autodiscovered gateway hubs")
            
            new_services = lambda l1,l2: [x for x in l1 if x not in l2]
            for service in new_services(resp.services,previously_found_hubs):
                previously_found_hubs.append(service)
                (ip, port) = rocon_gateway.resolveZeroconfAddress(service)
                rospy.loginfo("gw: discovered hub at " + str(ip) + ":"+str(service.port))
                try:
                    hub_name = rocon_gateway.resolveHub(ip,port)
                except redis.exceptions.ConnectionError:
                    rospy.logerr("gw: couldn't connect to the hub [%s:%s]", ip, port)
                    continue
                # Check blacklist (ip or hub name)
                if ip in self.param['blacklist']:
                    rospy.loginfo("gw: ignoring blacklisted hub [%s]",ip)
                    continue
                if hub_name in self.param['blacklist']:
                    rospy.loginfo("gw: ignoring blacklisted hub [%s]",hub_name)
                    continue
                # Handle whitelist (ip or hub name)
                if len(self.param['whitelist']) == 0:
                    if self.connect(service):
                        self.is_connected = True
                        break
                elif ip in self.param['whitelist']:
                    if self.connect(service):
                        self.is_connected = True
                        break
                else:
                    if hub_name in  self.param['whitelist']:
                        if self.connect(service):
                            self.is_connected = True
                            break
            rospy.sleep(3.0)

        # Once you get here, it is connected to redis server
        rospy.loginfo("gw: connected to hub.") 
        rospy.loginfo("Register default public topic/service")
        try:
            self.gateway_sync.addPublicTopics(self.param['local_public_topic'])
            self.gateway_sync.addPublicService(self.param['local_public_service'])
        except Exception as e:
            print str(e)
            sys.exit(0)
        rospy.spin()

        # When the node is going off, it should remove it's info from redis-server
        self.clearServer()
        


if __name__ == '__main__':
    
    rospy.init_node('multimaster_gateway')

    gateway = Gateway()
    rospy.loginfo("Initilized")

    gateway.spin()
    rospy.loginfo("Done")

