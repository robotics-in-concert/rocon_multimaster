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
from gateway_comms.msg import Rule
from utils import Connection

##############################################################################
# Master
##############################################################################

class LocalMaster(rosgraph.Master):
    '''
      Representing a ros master (local ros master). Just contains a 
      few utility methods for retrieving master related information as well
      as handles for registering and unregistering rules that have 
      been pulled or flipped in from another gateway.
    '''

    def __init__(self):
        rosgraph.Master.__init__(self,rospy.get_name())

    ##########################################################################
    # Registration
    ##########################################################################

    def register(self,registration):
        '''
          Registers a rule with the local master.
          
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
        if registration.connection_type == Rule.PUBLISHER:
            node_master.registerPublisher(registration.remote_name,registration.type_info,registration.xmlrpc_uri)
            return registration
        elif registration.connection_type == Rule.SUBSCRIBER:
            node_master.registerSubscriber(registration.remote_name,registration.type_info,registration.xmlrpc_uri)
            return registration
        elif registration.connection_type == Rule.SERVICE:
            node_master.registerService(registration.remote_name,registration.type_info,registration.xmlrpc_uri)
            return registration
        else:
            print registration
            rospy.logwarn("Gateway : you have discovered an empty stub for registering a local %s"%registration.connection_type)
            return None

    def unregister(self,registration):
        '''
          Unregisters a rule with the local master.
          
          @param registration : registration details for an existing gateway registered rule
          @type utils.Registration
        '''
        node_master = rosgraph.Master(registration.local_node)
        rospy.loginfo("Gateway : unregistering local node [%s] for [%s]"%(registration.local_node,registration.remote_name))
        if registration.connection_type == Rule.PUBLISHER:
            node_master.unregisterPublisher(registration.remote_name,registration.xmlrpc_uri)
        elif registration.connection_type == Rule.SUBSCRIBER:
            node_master.unregisterSubscriber(registration.remote_name,registration.xmlrpc_uri)
        elif registration.connection_type == Rule.SERVICE:
            node_master.unregisterService(registration.remote_name,registration.type_info)
        else:
            rospy.logwarn("Gateway : you have discovered an empty stub for registering a local %s"%registration.rule.type)
        
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
                    # remove action entries from publishers/subscribers
                    for connection in pubs:
                        if connection[0] in [base_topic + 'goal', base_topic + 'cancel']:
                            connection[1].remove(node)
                    for connection in subs:
                        if connection[0] in [base_topic + 'status', base_topic + 'feedback', base_topic + 'result']:
                            connection[1].remove(node)

        pubs[:] = [connection for connection in pubs if len(connection[1]) != 0]
        subs[:] = [connection for connection in subs if len(connection[1]) != 0]
        return actions, pubs, subs

    def getActionServers(self, publishers, subscribers):
        actions, subs, pubs = self._getActions(subscribers,publishers)
        return actions, pubs, subs

    def getActionClients(self, publishers, subscribers):
        actions, pubs, subs = self._getActions(publishers,subscribers)
        return actions, pubs, subs

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
                rule = Rule(type,topic_name,node)
                connection = Connection(rule, topic_type,node_uri)
                connections.append(connection)
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
                rule = Rule(type,action_name,node)
                connection = Connection(rule, topic_type, node_uri)
                connections.append(connection)
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
                rule = Rule(type,service_name,node)
                connection = Connection(rule, service_uri, node_uri)
                connections.append(connection)
        return connections

    def getConnectionState(self):
        connections = {}
        publishers, subscribers, services = self.getSystemState()
        action_servers, publishers, subscribers = self.getActionServers(publishers, subscribers)
        action_clients, publishers, subscribers = self.getActionClients(publishers, subscribers)
        connections[Rule.PUBLISHER] = self.getConnectionsFromPubSubList(publishers, Rule.PUBLISHER)
        connections[Rule.SUBSCRIBER] = self.getConnectionsFromPubSubList(subscribers, Rule.SUBSCRIBER)
        connections[Rule.SERVICE] = self.getConnectionsFromServiceList(services, Rule.SERVICE)
        connections[Rule.ACTION_SERVER] = self.getConnectionsFromActionList(action_servers, Rule.ACTION_SERVER)
        connections[Rule.ACTION_CLIENT] = self.getConnectionsFromActionList(action_clients, Rule.ACTION_CLIENT)
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
