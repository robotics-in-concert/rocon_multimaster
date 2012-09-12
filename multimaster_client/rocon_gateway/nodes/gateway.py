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
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Yujin Robot nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
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
  zeroconf_connection_service = "/zeroconf/list_discovered_services"

  # request from local node
  remote_topic_handler = "/gateway/topic"
  remote_service_handler = "/gateway/service"

  # Request foreign topic 
  request_topic_handler_name = "/gateway/foreign_topic_request"

  # Remote topic list request
  remote_topic_list_service_name = "/gateway/remotelist"
  gateway_sync = None
  param = {}
  is_connected = False
  allow_random_redis_server = False

  def __init__(self):
    self.parse_params()

    # Instantiate a GatewaySync module. This will take care of all redis server connection, communicatin with ros master uri
    self.gateway_sync = GatewaySync()

    # Subscribe from zero conf new connection
    rospy.loginfo("Wait for zeroconf service...")
    rospy.wait_for_service(self.zeroconf_connection_service)
    self.zeroconf_service_proxy = rospy.ServiceProxy(self.zeroconf_connection_service,ListDiscoveredServices)
    rospy.loginfo("Done")
    
    # Service Server for remote list request 
    self.remote_list_srv = rospy.Service(self.remote_topic_list_service_name,GetRemoteLists,self.processRemoteListRequest)

    # Service Server for public topic/service handler
    self.public_topic_handler = rospy.Service(self.remote_topic_handler,PublicHandler,self.processPublicTopicRequest)
    self.public_service_handler = rospy.Service(self.remote_service_handler,PublicHandler,self.processPublicServiceRequest)

    # Service Server for request foreign topic
    self.request_topic_handler = rospy.Service(self.request_topic_handler_name,PublicHandler,self.processRemoteTopicRequest)

  def parse_params(self):
    # Local topics and services to register redis server
    self.param['local_public_topic'] = rospy.get_param('~local_public_topic','')
    self.param['local_public_service'] = rospy.get_param('~local_public_service','')

    # Topics and services that need from remote server
    self.param['remote_topic'] = rospy.get_param('~remote_topic','')
    self.param['remoteservice'] = rospy.get_param('~remote_service','')
    self.param['white_list'] = rospy.get_param('~white_list','')
    self.param['black_list'] = rospy.get_param('~black_list','')

    # if both white_list and black_list are empty, it connects to any discovered redis server
    if len(self.param['white_list']) == 0:
      self.allow_random_redis_server = True

  def processPublicServiceRequest(self,request):
    success = False
    if request.command == "register":
      success = self.gateway_sync.addPublicService(request.list)
    elif request.command == "remove":
      success = self.gateway_sync.removePublicService(request.list)
    else:
      rospy.loginfo("Public Service Wrong command : " + request.command)
    
    return PublicHandlerResponse(success)

  def processPublicTopicRequest(self,request):
    success = False
    if request.command == "register":
      success = self.gateway_sync.addPublicTopics(request.list)
    elif request.command == "remove":
      success = self.gateway_sync.removePublicTopics(request.list)
    else:
      rospy.loginfo("Public Topic Wrong command : " + request.command)

    
    return PublicHandlerResponse(success)


  # This function receives a service request from local ros node, crawl remote topic/service list from redis, and respose to local ros node.
  def processRemoteListRequest(self,request):
    remote_list = self.gateway_sync.getRemoteLists()

    r = GetRemoteListsResponse()
    r.list = []

    for host in remote_list.keys():
      l = RemoteList()
      l.hostname = host
      l.topics = remote_list[host]['topic']
      l.services= remote_list[host]['service']
      r.list.append(l)
      
    return r

  def processRemoteTopicRequest(self,request):
    print str(request)
    try:
      if request.command == "register":
        self.gateway_sync.requestForeignTopic(request.list)
#      elif request.command == "remove":
#        self.gateway_sync.unregister_foreign_topic(request.list)
      else:
        rospy.loginfo("Error in RemoteTopicRequest : " + request.command)
    except Exception as e:
      print str(e)
      return PublicHandlerResponse(False)

    return PublicHandlerResponse(True)

  # It clears this client's information from redis-server
  def clearServer(self):
    try:
      self.gateway_sync.clearServer()
    except Exception as e:
      print str(e)

    print "Server cleared"


  def connect(self,msg):
    ip = "localhost"
    if not msg.is_local:
      ip = msg.ipv4_addresses[0]

    # Connects to Redis server
    if self.gateway_sync.connectToRedisServer(ip,msg.port):
      return True
    else:
      return False



  def spin(self):
    
    while not rospy.is_shutdown() and not self.is_connected:

      # Get discovered redis server list from zeroconf
      req = ListDiscoveredServicesRequest() 
      req.service_type = self.zeroconf_service
      resp = self.zeroconf_service_proxy(req)

      rospy.loginfo("Received available server")

      for service in resp.services:
        # if both white_list is empty, connect to any redis server that is not in black list
        ip = service.ipv4_addresses[0]
        rospy.loginfo("Redis Server is discovered = " + str(ip) + ":"+str(service.port))

        if len(self.param['white_list']) == 0 and ip not in self.param['black_list']:
          if self.connect(service):
            self.is_connected = True
            break
        # if white list is not empty, it only waits for ip in white_list 
        elif ip in self.param['white_list']:
          if self.connect(service):
            self.is_connected = True
            break

      rospy.loginfo("No valid redis server is up. Will try again..")
      rospy.sleep(3.0)

    # Once you get here, it is connected to redis server
    rospy.loginfo("Connected to Server") 

    rospy.spin()

    # When the node is going off, it should remove it's info from redis-server
    self.clearServer()
    


if __name__ == '__main__':
  
  rospy.init_node('multimaster_gateway')

  gateway = Gateway()
  rospy.loginfo("Initilized")

  gateway.spin()

