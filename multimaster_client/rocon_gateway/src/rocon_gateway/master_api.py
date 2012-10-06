#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway/LICENSE 
#

import roslib; roslib.load_manifest('rocon_gateway')
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
import re

from .exceptions import GatewayError, ConnectionTypeError
from .utils import Connection
from gateway_comms.msg import Connection as ConnectionMsg

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
    
    def _getMasterUri(self):
        return rosgraph.get_master_uri()

    def _isTopicNodeInList(self,topic,node,list):
        # check if cancel available
        available = False
        for candidate in list:
            if candidate[0] == topic and node in candidate[1]:
                available = True
                break
        return available

    def _getActions(self, pubs, subs):

        actions = []
        for goal_candidate in pubs:
            if re.search('goal$', goal_candidate[0]):
                # goal found, extract base topic
                base_topic = re.sub('goal$','',goal_candidate[0])
                nodes = goal_candidate[1]
                action_nodes = []

                # there may be multiple nodes -- for each node search for the other topics
                for node in nodes:
                    is_action = True
                    is_action &= self._isTopicNodeInList(base_topic + 'cancel',node,pubs)
                    is_action &= self._isTopicNodeInList(base_topic + 'status',node,subs)
                    is_action &= self._isTopicNodeInList(base_topic + 'feedback',node,subs)
                    is_action &= self._isTopicNodeInList(base_topic + 'result',node,subs)

                    if is_action:
                        action_nodes.append(node)

                if len(action_nodes) != 0:
                    # yay! an action has been found
                    actions.append([base_topic, action_nodes])
        return actions

    def getActionServers(self):
        publishers, subscribers, _ = self.getSystemState()
        return self._getActions(subscribers,publishers)

    def getActionClients(self):
        publishers, subscribers, _ = self.getSystemState()
        return self._getActions(publishers,subscribers)

    def getConnectionsFromPubSubList(self,list,type):
        connections = []
        for topic in list:
            topic_name = topic[0]
            topic_type = rostopic.get_topic_type(topic_name)
            topic_type = topic_type[0]
            nodes = topic[1]
            for node in nodes:
                node_uri = self.lookupNode(node)
                connections.append(Connection(type,topic_name,node,node_uri,None,topic_type))
        return connections

    def getConnectionsFromActionList(self,list,type):
        connections = []
        for action in list:
            action_name = action[0]
            goal_topic = action_name + 'goal'
            goal_topic_type = rostopic.get_topic_type(goal_topic)
            topic_type = re.sub('ActionGoal$', '', goal_topic_type[0]) #Base type for action
            nodes = action[1]
            for node in nodes:
                node_uri = self.lookupNode(node)
                connections.append(Connection(type,action_name,node,node_uri,None,topic_type))
        return connections

    def getConnectionsFromServiceList(self,list,type):
        connections = []
        for service in list:
            service_name = service[0]
            service_uri = rosservice.get_service_uri(service_name)
            nodes = service[1]
            for node in nodes:
                node_uri = self.lookupNode(node)
                connections.append(Connection(type,service_name,node,node_uri,service_uri,None))
        return connections

    def getConnectionState(self):
        connections = {}
        publishers, subscribers, services = self.getSystemState()
        action_servers = self.getActionServers()
        action_clients = self.getActionClients()
        connections[ConnectionMsg.PUBLISHER] = self.getConnectionsFromPubSubList(publishers, ConnectionMsg.PUBLISHER)
        connections[ConnectionMsg.SUBSCRIBER] = self.getConnectionsFromPubSubList(subscribers, ConnectionMsg.SUBSCRIBER)
        connections[ConnectionMsg.SERVICE] = self.getConnectionsFromServiceList(services, ConnectionMsg.SERVICE)
        connections[ConnectionMsg.ACTION_SERVER] = self.getConnectionsFromActionList(action_servers, ConnectionMsg.ACTION_SERVER)
        connections[ConnectionMsg.ACTION_CLIENT] = self.getConnectionsFromActionList(action_clients, ConnectionMsg.ACTION_CLIENT)
        return connections

    def _getAnonymousNodeName(self,topic):
        t = topic[1:len(topic)]
        name = roslib.names.anonymous_name(t)
        return name

    def _checkIfItisLocal(self,name,uri,identifier):
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
            if self._checkIfItisLocal(topic,uri,"topic"):
                rospy.logerr("Gateway : Topic triple available locally")
                return False
       
            node_name = self._getAnonymousNodeName(topic)    
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
            if self._checkIfItisLocal(service,node_xmlrpc_uri,"service"):
                rospy.logerr("Gateway : Service triple available locally")
                return False
            node_name = self._getAnonymousNodeName(service)    
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
        # (piyushk) commented out - will have explicit conn type
        # if connectionTypeString(connection) == "invalid":
        #     raise ConnectionTypeError("trying to register an invalid connection type [%s]"%connection)
        # components = connection.split(',')
        # if connectionTypeString(connection) == "topic":
        #     self.registerTopic(components[0],components[1],components[2])
        # elif connectionTypeString(connection) == "service":
        #     self.registerService(components[0],components[1],components[2])
        pass

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
        # (piyushk) commented out - will have explicit conn type
        # if connectionTypeString(connection) == "invalid":
        #     raise ConnectionTypeError("trying to unregister an invalid connection type [%s]"%connection)
        # components = connection.split(',')
        # if connectionTypeString(connection) == "topic":
        #     self.unregisterTopic(components[0],components[1],components[2])
        # elif connectionTypeString(connection) == "service":
        #     self.unregisterService(components[0],components[1],components[2])
        pass

    ##########################################################################
    # Other
    ##########################################################################

    def clear(self):
        self.pubs_node = {}
        self.pubs_uri = {}



