#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway/LICENSE 
#

##############################################################################
# Imports
##############################################################################

import roslib; roslib.load_manifest('rocon_gateway')
import rospy
import rosgraph
import rostopic
import rosservice
import rosnode
import roslib.names
#import itertools
import socket
import re

from .exceptions import GatewayError, ConnectionTypeError
from gateway_comms.msg import Connection

##############################################################################
# Master
##############################################################################

class LocalMaster(rosgraph.Master):
    '''
      Representing a ros master (local ros master). Just contains a 
      few utility methods for retrieving master related information as well
      as handles for registering and unregistering connections that have 
      been pulled or flipped in from another gateway.
    '''

    def __init__(self):
        rosgraph.Master.__init__(self,rospy.get_name())

    ##########################################################################
    # Registration
    ##########################################################################

    def register(self,registration):
        '''
          Registers a connection with the local master.
          
          @param registration : registration details
          @type utils.Registration
          
          @return the updated registration object (only adds an anonymously generated local node name)
          @rtype utils.Registration
        '''
        registration.local_node = self._getAnonymousNodeName(registration.remote_node)    
        rospy.loginfo("Gateway : registering a new node [%s] for [%s]"%(registration.local_node,registration.remote_name))
        
        # Then do we need checkIfIsLocal? Needs lots of parsing time, and the outer class should
        # already have handle that. 

        node_master = rosgraph.Master(registration.local_node)
        if registration.connection_type == Connection.PUBLISHER:
            node_master.registerPublisher(registration.remote_name,registration.type_info,registration.xmlrpc_uri)
            return registration
        elif registration.connection_type == Connection.SUBSCRIBER:
            node_master.registerSubscriber(registration.remote_name,registration.type_info,registration.xmlrpc_uri)
            return registration
        elif registration.connection_type == Connection.SERVICE:
            node_master.registerService(registration.remote_name,registration.type_info,registration.xmlrpc_uri)
            return registration
        else:
            print registration
            rospy.logwarn("Gateway : you have discovered an empty stub for registering a local %s"%registration.connection_type)
            return None

    def unregister(self,registration):
        '''
          Unregisters a connection with the local master.
          
          @param registration : registration details for an existing gateway registered connection
          @type utils.Registration
        '''
        node_master = rosgraph.Master(registration.local_node)
        rospy.loginfo("Gateway : unregistering local node [%s] for [%s]"%(registration.local_node,registration.remote_name))
        if registration.connection_type == Connection.PUBLISHER:
            node_master.unregisterPublisher(registration.remote_name,registration.xmlrpc_uri)
        elif registration.connection_type == Connection.SUBSCRIBER:
            node_master.unregisterSubscriber(registration.remote_name,registration.xmlrpc_uri)
        elif registration.connection_type == Connection.SERVICE:
            node_master.unregisterService(registration.remote_name,registration.type_info)
        else:
            rospy.logwarn("Gateway : you have discovered an empty stub for registering a local %s"%registration.connection.type)
        
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

    def getActionServers(self, publishers, subscribers):
        return self._getActions(subscribers,publishers)

    def getActionClients(self, publishers, subscribers):
        return self._getActions(publishers,subscribers)

    def getConnectionsFromPubSubList(self,list,type):
        connections = []
        for topic in list:
            topic_name = topic[0]
            topic_type = rostopic.get_topic_type(topic_name)
            topic_type = topic_type[0]
            nodes = topic[1]
            for node in nodes:
                try:
                    node_uri = self.lookupNode(node)
                except:
                    continue
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
                try:
                    node_uri = self.lookupNode(node)
                except:
                    continue
                connections.append(Connection(type,action_name,node,node_uri,None,topic_type))
        return connections

    def getConnectionsFromServiceList(self,list,type):
        connections = []
        for service in list:
            service_name = service[0]
            service_uri = rosservice.get_service_uri(service_name)
            nodes = service[1]
            for node in nodes:
                try:
                    node_uri = self.lookupNode(node)
                except:
                    continue
                connections.append(Connection(type,service_name,node,node_uri,service_uri,None))
        return connections

    def getConnectionState(self):
        connections = {}
        publishers, subscribers, services = self.getSystemState()
        action_servers = self.getActionServers(publishers, subscribers)
        action_clients = self.getActionClients(publishers, subscribers)
        connections[Connection.PUBLISHER] = self.getConnectionsFromPubSubList(publishers, Connection.PUBLISHER)
        connections[Connection.SUBSCRIBER] = self.getConnectionsFromPubSubList(subscribers, Connection.SUBSCRIBER)
        connections[Connection.SERVICE] = self.getConnectionsFromServiceList(services, Connection.SERVICE)
        connections[Connection.ACTION_SERVER] = self.getConnectionsFromActionList(action_servers, Connection.ACTION_SERVER)
        connections[Connection.ACTION_CLIENT] = self.getConnectionsFromActionList(action_clients, Connection.ACTION_CLIENT)
        return connections

    def _getAnonymousNodeName(self,topic):
        t = topic[1:len(topic)]
        name = roslib.names.anonymous_name(t)
        return name

##############################################################################
# Depracating
##############################################################################

#    def _checkIfItisLocal(self,name,uri,identifier):
#        pubs, _1, srvs = self.getSystemState()
#    
#        if identifier == "topic":
#            for p in itertools.chain(*[l for x, l in pubs]):
#                uri_m = rostopic.get_api(self,p)
#                if uri_m == uri:
#                    return True
#        elif identifier == "service":
#            nodename = rosservice.get_service_node(name)
#            if not nodename:
#                return False
#    
#            nodeuri = rosnode.get_api_uri(self,nodename)
#    
#            if nodeuri == uri:
#                return True
#        else:
#            print "Wrong Identifier in checkIfItisLocal"
#        return False
