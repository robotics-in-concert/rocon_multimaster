#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway/LICENSE 
# Copyright (c) 2012, Yujin Robot, Daniel Stonier
#

import roslib; roslib.load_manifest('rocon_gateway')
import rospy
import rosgraph
import rocon_gateway
from gateway_comms.msg import *
from gateway_comms.srv import *
from zeroconf_comms.srv import *
from rocon_gateway_sync import *
from std_msgs.msg import String
from urlparse import urlparse


# This class is wrapper ros class of gateway sync.
# The role of this node is below
# 1. listens to server up/down status from zero configuration node
# 2. listens to local ros node's remote topic registration request
# 3. response a local ros node's get remote topic/service list request
class Gateway():


    def __init__(self):
        # Gateway configuration
        self.zeroconf = False
        self.zeroconf_service = "_ros-gateway-hub._tcp"
        self.gateway_sync = GatewaySync() # redis server (hub) and local ros master connections
        self.is_connected = False # used to flag whether it is connected to the hub (move to gateway sync)
        self.param = rocon_gateway.rosParameters()
        self.callbacks = {}
        
        # Topic and Service Names
        gateway_info_service_name = "~info"
        gateway_request_service_name = "~request"
        gateway_connect_subscriber_name = "~connect"
        zeroconf_add_listener_service = "zeroconf/add_listener"
        zeroconf_connection_service = "zeroconf/list_discovered_services"
        
        # Initialisation variables
        zeroconf_timeout = 5 # Amount of time to wait for the zeroconf services to appear

        self.setupCallbacks()

        # Service Server for local node requests
        self.remote_list_service = rospy.Service(gateway_request_service_name,PublicHandler,self.processLocalRequest)

        self.connect_hub_subscriber = rospy.Subscriber(gateway_connect_subscriber_name,String,self.processConnectHubRequest)

        # Service Server for gateway info
        self.gateway_info_service = rospy.Service(gateway_info_service_name,GatewayInfo,self.processGatewayInfo)

        if self.param['hub_uri'] != '':
            if self.connectByUri(self.param['hub_uri']):
                rospy.logwarn("Gateway : made direct connection attempt to hub [%s]"%self.param['hub_uri'])
                self.is_connected = True
            else:
                rospy.logwarn("Gateway : failed direct connection attempt to hub [%s]"%self.param['hub_uri'])
        else: # see if we can use zeroconf to autodect
            rospy.loginfo("Gateway : waiting for zeroconf service to come up...")
            try:
                rospy.wait_for_service(zeroconf_add_listener_service, timeout=zeroconf_timeout)
                self.zeroconf = True
            except rospy.ROSException:
                rospy.logwarn("Gateway : timed out waiting for zeroconf services to come up.")
                
            if self.zeroconf:
                zeroconf_add_listener = rospy.ServiceProxy(zeroconf_add_listener_service,AddListener)
                self.zeroconf_service_proxy = rospy.ServiceProxy(zeroconf_connection_service,ListDiscoveredServices)
                if not zeroconf_add_listener(service_type = self.zeroconf_service):
                    self.zeroconf = False

    def setupCallbacks(self):
        self.callbacks["get_public_interfaces"] = self.processRemoteListRequest
        self.callbacks["add_public_topic"] = self.gateway_sync.addPublicTopics
        self.callbacks["remove_public_topic"] = self.gateway_sync.removePublicTopics

        self.callbacks["add_named_topics"] = self.gateway_sync.addNamedTopics
        self.callbacks["remove_named_topics"] = self.gateway_sync.removeNamedTopics

        self.callbacks["add_public_service"] = self.gateway_sync.addPublicService
        self.callbacks["remove_public_service"] = self.gateway_sync.removePublicService
        self.callbacks["add_named_services"] = self.gateway_sync.addNamedServices
        self.callbacks["remove_named_services"] = self.gateway_sync.removeNamedServices

        self.callbacks["register_foreign_topic"] = self.gateway_sync.requestForeignTopic
        self.callbacks["unregister_foreign_topic"] = self.gateway_sync.unregisterForeignTopic

        self.callbacks["register_foreign_service"] = self.gateway_sync.requestForeignService
        self.callbacks["unregister_foreign_service"] = self.gateway_sync.unregisterForeignService

        self.callbacks["make_all_public"] = self.gateway_sync.makeAllPublic
        self.callbacks["remove_all_public"] = self.gateway_sync.removeAllPublic
     
        self.callbacks["flipout_topic"] = self.flipoutTopic
        self.callbacks["flipout_service"] = self.flipoutService

        self.callbacks["post"] = self.gateway_sync.post


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

    def processConnectHubRequest(self,uri):
        '''
          Incoming requests are used to then try and connect to the gateway hub
          if not already connected.
          
          Requests are of the form of a uri (hostname:port pair) pointing to 
          the gateway hub. 
        '''
        if not self.is_connected:
            if self.connectByUri(uri.data):
                rospy.logwarn("Gateway : made direct connection attempt to hub [%s]"%uri.data)
                self.is_connected = True
            else:
                rospy.logwarn("Gateway : failed direct connection attempt to hub [%s]"%uri.data)
        else:
            rospy.logwarn("Gateway : is already connected to a hub, cowardly refusing to connect.")

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


    def connectByZeroconfName(self,msg):
        (ip, port) = rocon_gateway.resolveZeroconfAddress(msg)
        return self.connect(ip,port)
        
    def connectByUri(self,uri):
        o = urlparse(uri)
        return self.connect(o.hostname, o.port)
    
    def connect(self,ip,port):
        if self.gateway_sync.connectToRedisServer(ip,port):
            return True
        else:
            return False

    def spin(self):
        previously_found_hubs = []
        while not rospy.is_shutdown() and not self.is_connected:
            if self.zeroconf:
                # Get discovered redis server list from zeroconf
                req = ListDiscoveredServicesRequest() 
                req.service_type = self.zeroconf_service
                resp = self.zeroconf_service_proxy(req)
                
                rospy.logdebug("Gateway : checking for autodiscovered gateway hubs")
                
                new_services = lambda l1,l2: [x for x in l1 if x not in l2]
                for service in new_services(resp.services,previously_found_hubs):
                    previously_found_hubs.append(service)
                    (ip, port) = rocon_gateway.resolveZeroconfAddress(service)
                    rospy.loginfo("Gateway : discovered hub at " + str(ip) + ":"+str(service.port))
                    try:
                        hub_name = rocon_gateway.resolveHub(ip,port)
                        rospy.loginfo("Gateway : hub name [%s]", hub_name)
                    except redis.exceptions.ConnectionError:
                        rospy.logerr("Gateway : couldn't connect to the hub [%s:%s]", ip, port)
                        continue
                    # Check blacklist (ip or hub name)
                    if ip in self.param['hub_blacklist']:
                        rospy.loginfo("Gateway : ignoring blacklisted hub [%s]",ip)
                        continue
                    if hub_name in self.param['hub_blacklist']:
                        rospy.loginfo("Gateway : ignoring blacklisted hub [%s]",hub_name)
                        continue
                    # Handle whitelist (ip or hub name)
                    if len(self.param['hub_whitelist']) == 0:
                        if self.connectByZeroconfName(service):
                            self.is_connected = True
                            break
                    elif ip in self.param['hub_whitelist']:
                        if self.connectByZeroconfName(service):
                            self.is_connected = True
                            break
                    else:
                        if hub_name in self.param['hub_whitelist']:
                            if self.connectByZeroconfName(service):
                                self.is_connected = True
                                break
            else:
                rospy.logdebug("Gateway : waiting for hub uri input.")
                pass # add ip connect here
            rospy.sleep(3.0)

        # Once you get here, it is connected to redis server
        rospy.loginfo("Gateway : connected to hub.") 
        rospy.loginfo("Register default public topic/service")

        # Add public topics and services
        try:
            self.gateway_sync.addPublicTopics(self.param['local_public_topic'])
            self.gateway_sync.addPublicService(self.param['local_public_service'])
        except Exception as e:
            print str(e)
            sys.exit(0)

        # Add named public topics and services
        if self.param['public_named_topics']:
            self.gateway_sync.topic_whitelist.extend(self.param['public_named_topics'].split(','))
        if self.param['public_named_topics_blacklist']:
            self.gateway_sync.topic_blacklist.extend(self.param['public_named_topics_blacklist'].split(','))
        if self.param['public_named_services']:
            self.gateway_sync.service_whitelist.extend(self.param['public_named_services'].split(','))
        if self.param['public_named_services_blacklist']:
            self.gateway_sync.service_blacklist.extend(self.param['public_named_services_blacklist'].split(','))

        rospy.spin()

        # When the node is going off, it should remove it's info from redis-server
        self.clearServer()
        

if __name__ == '__main__':
    
    rospy.init_node('gateway')

    gateway = Gateway()
    rospy.loginfo("Gateway : initialised.")

    gateway.spin()
    rospy.loginfo("Gateway : shutting down.")

