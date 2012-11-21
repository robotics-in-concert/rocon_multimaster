#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway/LICENSE 
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
import re

from gateway_msgs.msg import Rule, ConnectionType
from utils import Connection, action_types

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
        registration.local_node = self._getAnonymousNodeName(registration.connection.rule.node)    
        rospy.loginfo("Gateway : registering a new node [%s] for [%s]"%(registration.local_node,registration))
        
        # Then do we need checkIfIsLocal? Needs lots of parsing time, and the outer class should
        # already have handle that.

        node_master = rosgraph.Master(registration.local_node)
        if registration.connection.rule.type == ConnectionType.PUBLISHER:
            node_master.registerPublisher(registration.connection.rule.name,registration.connection.type_info,registration.connection.xmlrpc_uri)
            return registration
        elif registration.connection.rule.type == ConnectionType.SUBSCRIBER:
            node_master.registerSubscriber(registration.connection.rule.name,registration.connection.type_info,registration.connection.xmlrpc_uri)
            return registration
        elif registration.connection.rule.type == ConnectionType.SERVICE:
            if rosservice.get_service_node(registration.connection.rule.name):
                rospy.logwarn("Gateway : tried to register a service that is already locally available, aborting [%s]"%registration.connection.rule.name)
                return None
            else:
                node_master.registerService(registration.connection.rule.name,registration.connection.type_info,registration.connection.xmlrpc_uri)
                return registration
        elif registration.connection.rule.type == ConnectionType.ACTION_SERVER:
            node_master.registerSubscriber(registration.connection.rule.name+"/goal",registration.connection.type_info+"ActionGoal",registration.connection.xmlrpc_uri)
            node_master.registerSubscriber(registration.connection.rule.name+"/cancel","actionlib_msgs/GoalID",registration.connection.xmlrpc_uri)
            node_master.registerPublisher(registration.connection.rule.name+"/status","actionlib_msgs/GoalStatusArray",registration.connection.xmlrpc_uri)
            node_master.registerPublisher(registration.connection.rule.name+"/feedback",registration.connection.type_info+"ActionFeedback",registration.connection.xmlrpc_uri)
            node_master.registerPublisher(registration.connection.rule.name+"/result",registration.connection.type_info+"ActionResult",registration.connection.xmlrpc_uri)
            return registration
        elif registration.connection.rule.type == ConnectionType.ACTION_CLIENT:
            node_master.registerPublisher(registration.connection.rule.name+"/goal",registration.connection.type_info+"ActionGoal",registration.connection.xmlrpc_uri)
            node_master.registerPublisher(registration.connection.rule.name+"/cancel","actionlib_msgs/GoalID",registration.connection.xmlrpc_uri)
            node_master.registerSubscriber(registration.connection.rule.name+"/status","actionlib_msgs/GoalStatusArray",registration.connection.xmlrpc_uri)
            node_master.registerSubscriber(registration.connection.rule.name+"/feedback",registration.connection.type_info+"ActionFeedback",registration.connection.xmlrpc_uri)
            node_master.registerSubscriber(registration.connection.rule.name+"/result",registration.connection.type_info+"ActionResult",registration.connection.xmlrpc_uri)
            return registration
        return None

    def unregister(self,registration):
        '''
          Unregisters a rule with the local master.
          
          @param registration : registration details for an existing gateway registered rule
          @type utils.Registration
        '''
        node_master = rosgraph.Master(registration.local_node)
        rospy.loginfo("Gateway : unregistering local node [%s] for [%s]"%(registration.local_node,registration))
        if registration.connection.rule.type == ConnectionType.PUBLISHER:
            node_master.unregisterPublisher(registration.connection.rule.name,registration.connection.xmlrpc_uri)
        elif registration.connection.rule.type == ConnectionType.SUBSCRIBER:
            node_master.unregisterSubscriber(registration.connection.rule.name,registration.connection.xmlrpc_uri)
        elif registration.connection.rule.type == ConnectionType.SERVICE:
            node_master.unregisterService(registration.connection.rule.name,registration.connection.type_info)
        elif registration.connection.rule.type == ConnectionType.ACTION_SERVER:
            node_master.unregisterSubscriber(registration.connection.rule.name+"/goal",registration.connection.xmlrpc_uri)
            node_master.unregisterSubscriber(registration.connection.rule.name+"/cancel",registration.connection.xmlrpc_uri)
            node_master.unregisterPublisher(registration.connection.rule.name+"/status",registration.connection.xmlrpc_uri)
            node_master.unregisterPublisher(registration.connection.rule.name+"/feedback",registration.connection.xmlrpc_uri)
            node_master.unregisterPublisher(registration.connection.rule.name+"/result",registration.connection.xmlrpc_uri)
        elif registration.connection.rule.type == ConnectionType.ACTION_CLIENT:
            node_master.unregisterPublisher(registration.connection.rule.name+"/goal",registration.connection.xmlrpc_uri)
            node_master.unregisterPublisher(registration.connection.rule.name+"/cancel",registration.connection.xmlrpc_uri)
            node_master.unregisterSubscriber(registration.connection.rule.name+"/status",registration.connection.xmlrpc_uri)
            node_master.unregisterSubscriber(registration.connection.rule.name+"/feedback",registration.connection.xmlrpc_uri)
            node_master.unregisterSubscriber(registration.connection.rule.name+"/result",registration.connection.xmlrpc_uri)
        
    ##########################################################################
    # Master utility methods
    ##########################################################################
    
    def generateConnectionDetails(self, type, name, node):
        '''
        Creates all the extra details to create a connection object from a
        rule.
        
        @param type : the connection type (one of gateway_msgs.msg.ConnectionType)
        @type string
        @param name : the name of the connection
        @type string
        @param node : the master node name it comes from
        @param string
        
        @return the utils.Connection object complete with type_info and xmlrpc_uri
        @type utils.Connection
        '''
        xmlrpc_uri = self.lookupNode(node)
        connections = []
        if type == ConnectionType.PUBLISHER or type == ConnectionType.SUBSCRIBER:
            type_info = rostopic.get_topic_type(name)[0] # message type
            connections.append(Connection(Rule(type, name, node),type_info,xmlrpc_uri))
        elif type == ConnectionType.SERVICE:
            type_info = rosservice.get_service_uri(name)
            connections.append(Connection(Rule(type, name, node),type_info,xmlrpc_uri))
        elif type == ConnectionType.ACTION_SERVER:
            type_info = rostopic.get_topic_type(name+'/goal')[0] # message type
            connections.append(Connection(Rule(ConnectionType.SUBSCRIBER, name+'/goal', node),type_info,xmlrpc_uri))
            type_info = rostopic.get_topic_type(name+'/cancel')[0] # message type
            connections.append(Connection(Rule(ConnectionType.SUBSCRIBER, name+'/cancel', node),type_info,xmlrpc_uri))
            type_info = rostopic.get_topic_type(name+'/status')[0] # message type
            connections.append(Connection(Rule(ConnectionType.PUBLISHER, name+'/status', node),type_info,xmlrpc_uri))
            type_info = rostopic.get_topic_type(name+'/feedback')[0] # message type
            connections.append(Connection(Rule(ConnectionType.PUBLISHER, name+'/feedback', node),type_info,xmlrpc_uri))
            type_info = rostopic.get_topic_type(name+'/result')[0] # message type
            connections.append(Connection(Rule(ConnectionType.PUBLISHER, name+'/result', node),type_info,xmlrpc_uri))
        elif type == ConnectionType.ACTION_CLIENT:
            type_info = rostopic.get_topic_type(name+'/goal')[0] # message type
            connections.append(Connection(Rule(ConnectionType.PUBLISHER, name+'/goal', node),type_info,xmlrpc_uri))
            type_info = rostopic.get_topic_type(name+'/cancel')[0] # message type
            connections.append(Connection(Rule(ConnectionType.PUBLISHER, name+'/cancel', node),type_info,xmlrpc_uri))
            type_info = rostopic.get_topic_type(name+'/status')[0] # message type
            connections.append(Connection(Rule(ConnectionType.SUBSCRIBER, name+'/status', node),type_info,xmlrpc_uri))
            type_info = rostopic.get_topic_type(name+'/feedback')[0] # message type
            connections.append(Connection(Rule(ConnectionType.SUBSCRIBER, name+'/feedback', node),type_info,xmlrpc_uri))
            type_info = rostopic.get_topic_type(name+'/result')[0] # message type
            connections.append(Connection(Rule(ConnectionType.SUBSCRIBER, name+'/result', node),type_info,xmlrpc_uri))
        return connections
    
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
                base_topic = re.sub('\/goal$','',goal_candidate[0])
                nodes = goal_candidate[1]
                action_nodes = []

                # there may be multiple nodes -- for each node search for the other topics
                for node in nodes:
                    is_action = True
                    is_action &= self._isTopicNodeInList(base_topic + '/cancel',node,pubs)
                    is_action &= self._isTopicNodeInList(base_topic + '/status',node,subs)
                    is_action &= self._isTopicNodeInList(base_topic + '/feedback',node,subs)
                    is_action &= self._isTopicNodeInList(base_topic + '/result',node,subs)

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
            goal_topic = action_name + '/goal'
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
        connections[ConnectionType.PUBLISHER] = self.getConnectionsFromPubSubList(publishers, ConnectionType.PUBLISHER)
        connections[ConnectionType.SUBSCRIBER] = self.getConnectionsFromPubSubList(subscribers, ConnectionType.SUBSCRIBER)
        connections[ConnectionType.SERVICE] = self.getConnectionsFromServiceList(services, ConnectionType.SERVICE)
        connections[ConnectionType.ACTION_SERVER] = self.getConnectionsFromActionList(action_servers, ConnectionType.ACTION_SERVER)
        connections[ConnectionType.ACTION_CLIENT] = self.getConnectionsFromActionList(action_clients, ConnectionType.ACTION_CLIENT)
        return connections

    def _getAnonymousNodeName(self,topic):
        t = topic[1:len(topic)]
        name = roslib.names.anonymous_name(t)
        return name

