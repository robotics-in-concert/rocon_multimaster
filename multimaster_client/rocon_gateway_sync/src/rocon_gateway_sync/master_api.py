#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway_sync/LICENSE 
#

import roslib; roslib.load_manifest('rocon_gateway_sync')
import rospy
import rosgraph.masterapi
import time
import rostopic
import rosservice
import rosnode
import roslib.names
import itertools
import socket
import os
import threading

from .exceptions import GatewayError, GatewayException, ConnectionTypeError
from .utils import Connection, connectionTypeString, connectionType

class LocalMaster(rosgraph.Master):
    '''
      Representing a ros master (local ros master). Just contains a 
      few utility methods for retrieving master related information as well
      as handles for registering and unregistering connections that have 
      been pulled or flipped in from another gateway.
    '''

    def __init__(self):
        rosgraph.Master.__init__(self,rospy.get_name())
        rospy.loginfo("Gateway: initialising ros manager")
    
        self.pubs_uri = {}
        self.pubs_node = {}
        self.srvs_uri = {}
        self.srvs_node = {}
        self.cv = threading.Condition()

    ##########################################################################
    # Master utility methods
    ##########################################################################
    
    def getMasterUri(self):
        return rosgraph.get_master_uri()

    def getTopicInfo(self,topic):
        infolist = []
        topictype, _1, _2 = rostopic.get_topic_type(topic)
        try:
            pubs, _1, _2 = self.getSystemState()
            pubs = [x for x in pubs if x[0] == topic]

            if not pubs:
                raise Exception("Unknown topic %s"%topic)

            for p in itertools.chain(*[l for x, l in pubs]):
                info = topictype + "," + self.lookupNode(p)
                infolist.append(info)
      
        except Exception as e:
            print str(e)
            raise e

        return infolist

    def getServiceInfo(self,service):
        try:
            info = ""
            srvuri = rosservice.get_service_uri(service)
            nodename = rosservice.get_service_node(service)
            nodeuri = rosnode.get_api_uri(self,nodename)
            info = srvuri + "," + nodeuri
        except Exception as e:
            print "Error in getServiceInfo"
            raise e

        return info

    def getAnonymousNodeName(self,topic):
        t = topic[1:len(topic)]
        name = roslib.names.anonymous_name(t)
        return name

    def checkIfItisLocal(self,name,uri,identifier):
        pubs, _1, srvs = self.getSystemState()
    
        if identifier == "topic":
            for p in itertools.chain(*[l for x, l in pubs]):
                uri_m = rostopic.get_api(self,p)
                if uri_m == uri:
                    return True
        elif identifier == "service":
            nodename = rosservice.get_service_node(name)
        
            if not nodename:
                return False
    
            nodeuri = rosnode.get_api_uri(self,nodename)
    
            if nodeuri == uri:
                return True
        else:
            print "Wrong Identifier in checkIfItisLocal"
        return False

    ##########################################################################
    # Pull methods - register/unregister pulls/unpulls with local master
    ##########################################################################
    
    def registerTopic(self,topic,topictype,uri):
        try:
            if self.checkIfItisLocal(topic,uri,"topic"):
                rospy.logerr("Gateway : Topic triple available locally")
                return False
       
            node_name = self.getAnonymousNodeName(topic)    
            rospy.logerr("Gateway : Starting new node [%s] for topic [%s]"%(node_name,topic))

            # Initialize if it is a new topic
            if topic not in self.pubs_uri.keys():
                self.pubs_uri[topic] = [] 
        
        
            self.cv.acquire()
            if uri  not in self.pubs_uri[topic]:
                self.pubs_uri[topic].append(uri)
                self.pubs_node[(topic,uri)] = node_name
                master = rosgraph.Master(node_name)
                master.registerPublisher(topic,topictype,uri)
            else:
                print "already registered"
                return False
            self.cv.release()
        except Exception as e:
            print "In registerTopic"
            raise

        return True

    def registerService(self,service,service_api,node_xmlrpc_uri):
        try:                                                  
            if self.checkIfItisLocal(service,node_xmlrpc_uri,"service"):
                rospy.logerr("Gateway : Service triple available locally")
                return False
            node_name = self.getAnonymousNodeName(service)    
            rospy.loginfo("Gateway : Starting new node [%s] for service [%s]"%(node_name,service))

            # Initialize if it is a new topic
            if service not in self.srvs_uri.keys():
                self.srvs_uri[service] = [] 
        
            self.cv.acquire()
            if service_api not in self.srvs_uri[service]:
                self.srvs_uri[service].append(service_api)
                self.srvs_node[(service,service_api)] =node_name
                master = rosgraph.Master(node_name)
                master.registerService(service,service_api,node_xmlrpc_uri)
            else:
                print "already registered"
                return False
            self.cv.release()
        except Exception as e:
            print "registerService:ros_master.py"
            raise

        return True

    def register(self,connection):
        if connectionType(connection) == Connection.invalid:
            raise ConnectionTypeError("trying to register an invalid connection type [%s]"%connection)
        components = connection.split(',')
        if connectionType(connection) == Connection.topic:
            self.registerTopic(components[0],components[1],components[2])
        elif connectionType(connection) == Connection.service:
            self.registerService(components[0],components[1],components[2])

    def unregisterTopic(self,topic,topictype,node_uri):
        try:
            try: 
                node_name = self.pubs_node[(topic,node_uri)]
            except KeyError:
                print "Topic does not exist"
                return False
            print "Unregistering ",topic," from ",node_name
            master_n = rosgraph.Master(node_name)
            master_n.unregisterPublisher(topic,node_uri)
            del self.pubs_node[(topic,node_uri)]
            self.pubs_uri[topic].remove(node_uri)
        except:
            print "Failed in unregister Topic"
            raise
        return True

    def unregisterService(self,service,service_api,node_uri):
        try:
            try: 
                node_name = self.srvs_node[(service,service_api)]
            except KeyError:
                print "Service does not exist"
                return False
            print "Unregistering ",service," from ",node_name
            master_n = rosgraph.Master(node_name)
            master_n.unregisterService(service, service_api)
            del self.srvs_node[(service,service_api)]
            self.srvs_uri[service].remove(service_api)
        except:
            print "Failed in unregister Service"
            raise
        return True

    def unregister(self,connection):
        if connectionType(connection) == Connection.invalid:
            raise ConnectionTypeError("trying to unregister an invalid connection type [%s]"%connection)
        components = connection.split(',')
        if connectionType(connection) == Connection.topic:
            self.unregisterTopic(components[0],components[1],components[2])
        elif connectionType(connection) == Connection.service:
            self.unregisterService(components[0],components[1],components[2])

    ##########################################################################
    # Other
    ##########################################################################

    def clear(self):
        self.pubs_node = {}
        self.pubs_uri = {}



