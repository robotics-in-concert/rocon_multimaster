#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway/LICENSE
#

##############################################################################
# Imports
##############################################################################

import os
import socket
import errno
import rospy
import rosgraph
import rostopic
import rosservice
import roslib.names
from rosmaster.util import xmlrpcapi
try:
    import urllib.parse as urlparse  # Python 3.x
except ImportError:
    import urlparse
import re
from gateway_msgs.msg import Rule, ConnectionType
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
        rosgraph.Master.__init__(self, rospy.get_name())

    ##########################################################################
    # Registration
    ##########################################################################

    def register(self, registration):
        '''
          Registers a rule with the local master.

          @param registration : registration details
          @type utils.Registration

          @return the updated registration object (only adds an anonymously generated local node name)
          @rtype utils.Registration
        '''
        registration.local_node = self._get_anonymous_node_name(registration.connection.rule.node)
        rospy.logdebug("Gateway : registering a new node [%s] for [%s]" % (registration.local_node, registration))

        # Then do we need checkIfIsLocal? Needs lots of parsing time, and the outer class should
        # already have handle that.

        node_master = rosgraph.Master(registration.local_node)
        if registration.connection.rule.type == ConnectionType.PUBLISHER:
            node_master.registerPublisher(registration.connection.rule.name, registration.connection.type_info, registration.connection.xmlrpc_uri)
            return registration
        elif registration.connection.rule.type == ConnectionType.SUBSCRIBER:
            self._register_subscriber(node_master, registration.connection.rule.name, registration.connection.type_info, registration.connection.xmlrpc_uri)
            return registration
        elif registration.connection.rule.type == ConnectionType.SERVICE:
            if rosservice.get_service_node(registration.connection.rule.name):
                rospy.logwarn("Gateway : tried to register a service that is already locally available, aborting [%s]" % registration.connection.rule.name)
                return None
            else:
                node_master.registerService(registration.connection.rule.name, registration.connection.type_info, registration.connection.xmlrpc_uri)
                return registration
        elif registration.connection.rule.type == ConnectionType.ACTION_SERVER:
            # Need to update these with self._register_subscriber
            self._register_subscriber(node_master, registration.connection.rule.name + "/goal", registration.connection.type_info + "ActionGoal", registration.connection.xmlrpc_uri)
            self._register_subscriber(node_master, registration.connection.rule.name + "/cancel", "actionlib_msgs/GoalID", registration.connection.xmlrpc_uri)
            node_master.registerPublisher(registration.connection.rule.name + "/status", "actionlib_msgs/GoalStatusArray", registration.connection.xmlrpc_uri)
            node_master.registerPublisher(registration.connection.rule.name + "/feedback", registration.connection.type_info + "ActionFeedback", registration.connection.xmlrpc_uri)
            node_master.registerPublisher(registration.connection.rule.name + "/result", registration.connection.type_info + "ActionResult", registration.connection.xmlrpc_uri)
            return registration
        elif registration.connection.rule.type == ConnectionType.ACTION_CLIENT:
            node_master.registerPublisher(registration.connection.rule.name + "/goal", registration.connection.type_info + "ActionGoal", registration.connection.xmlrpc_uri)
            node_master.registerPublisher(registration.connection.rule.name + "/cancel", "actionlib_msgs/GoalID", registration.connection.xmlrpc_uri)
            self._register_subscriber(node_master, registration.connection.rule.name + "/status", "actionlib_msgs/GoalStatusArray", registration.connection.xmlrpc_uri)
            self._register_subscriber(node_master, registration.connection.rule.name + "/feedback", registration.connection.type_info + "ActionFeedback", registration.connection.xmlrpc_uri)
            self._register_subscriber(node_master, registration.connection.rule.name + "/result", registration.connection.type_info + "ActionResult", registration.connection.xmlrpc_uri)
            return registration
        return None

    def unregister(self, registration):
        '''
          Unregisters a rule with the local master.

          @param registration : registration details for an existing gateway registered rule
          @type utils.Registration
        '''
        node_master = rosgraph.Master(registration.local_node)
        rospy.logdebug("Gateway : unregistering local node [%s] for [%s]" % (registration.local_node, registration))
        if registration.connection.rule.type == ConnectionType.PUBLISHER:
            node_master.unregisterPublisher(registration.connection.rule.name, registration.connection.xmlrpc_uri)
        elif registration.connection.rule.type == ConnectionType.SUBSCRIBER:
            self._unregister_subscriber(node_master, registration.connection.xmlrpc_uri, registration.connection.rule.name)
        elif registration.connection.rule.type == ConnectionType.SERVICE:
            node_master.unregisterService(registration.connection.rule.name, registration.connection.type_info)
        elif registration.connection.rule.type == ConnectionType.ACTION_SERVER:
            self._unregister_subscriber(node_master, registration.connection.xmlrpc_uri, registration.connection.rule.name + "/goal")
            self._unregister_subscriber(node_master, registration.connection.xmlrpc_uri, registration.connection.rule.name + "/cancel")
            node_master.unregisterPublisher(registration.connection.rule.name + "/status", registration.connection.xmlrpc_uri)
            node_master.unregisterPublisher(registration.connection.rule.name + "/feedback", registration.connection.xmlrpc_uri)
            node_master.unregisterPublisher(registration.connection.rule.name + "/result", registration.connection.xmlrpc_uri)
        elif registration.connection.rule.type == ConnectionType.ACTION_CLIENT:
            node_master.unregisterPublisher(registration.connection.rule.name + "/goal", registration.connection.xmlrpc_uri)
            node_master.unregisterPublisher(registration.connection.rule.name + "/cancel", registration.connection.xmlrpc_uri)
            self._unregister_subscriber(node_master, registration.connection.xmlrpc_uri, registration.connection.rule.name + "/status")
            self._unregister_subscriber(node_master, registration.connection.xmlrpc_uri, registration.connection.rule.name + "/feedback")
            self._unregister_subscriber(node_master, registration.connection.xmlrpc_uri, registration.connection.rule.name + "/result")

    def _register_subscriber(self, node_master, name, type_info, xmlrpc_uri):
        '''
          This one is not necessary, since you can pretty much guarantee the
          existence of the subscriber here, but it pays to be safe - we've seen
          some errors come out here when the ROS_MASTER_URI was only set to
          localhost.

          @param node_master : node-master xmlrpc method handler
          @param type_info : type of the subscriber message
          @param xmlrpc_uri : the uri of the node (xmlrpc server)
          @type string
          @param name : fully resolved subscriber name
        '''
        # This unfortunately is a game breaker - it destroys all connections, not just those
        # connected to this master, see #125.
        pub_uri_list = node_master.registerSubscriber(name, type_info, xmlrpc_uri)
        try:
            #rospy.loginfo("register_subscriber [%s][%s][%s]" % (name, xmlrpc_uri, pub_uri_list))
            xmlrpcapi(xmlrpc_uri).publisherUpdate('/master', name, pub_uri_list)
        except socket.error, v:
            errorcode = v[0]
            if errorcode != errno.ECONNREFUSED:
                rospy.logerr("Gateway : error registering subscriber (is ROS_MASTER_URI and ROS_HOSTNAME or ROS_IP correctly set?)")
                rospy.logerr("Gateway : errorcode [%s] xmlrpc_uri [%s]" % (str(errorcode), xmlrpc_uri))
                raise  # better handling here would be ideal
            else:
                pass  # subscriber stopped on the other side, don't worry about it

    def _unregister_subscriber(self, node_master, xmlrpc_uri, name):
        '''
          It is a special case as it requires xmlrpc handling to inform the subscriber of
          the disappearance of publishers it was connected to. It also needs to handle the
          case when that information doesn't get to the subscriber because it is down.

          @param node_master : node-master xmlrpc method handler
          @param xmlrpc_uri : the uri of the node (xmlrpc server)
          @type string
          @param name : fully resolved subscriber name
        '''
        # This unfortunately is a game breaker - it destroys all connections, not just those
        # connected to this master, see #125.
        try:
            xmlrpcapi(xmlrpc_uri).publisherUpdate('/master', name, [])
        except socket.error, v:
            errorcode = v[0]
            if errorcode != errno.ECONNREFUSED:
                raise  # better handling here would be ideal
            else:
                pass  # subscriber stopped on the other side, don't worry about it
        node_master.unregisterSubscriber(name, xmlrpc_uri)

    ##########################################################################
    # Master utility methods
    ##########################################################################

    def generate_connection_details(self, type, name, node):
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
            type_info = rostopic.get_topic_type(name)[0]  # message type
            connections.append(Connection(Rule(type, name, node), type_info, xmlrpc_uri))
        elif type == ConnectionType.SERVICE:
            type_info = rosservice.get_service_uri(name)
            connections.append(Connection(Rule(type, name, node), type_info, xmlrpc_uri))
        elif type == ConnectionType.ACTION_SERVER:
            type_info = rostopic.get_topic_type(name + '/goal')[0]  # message type
            connections.append(Connection(Rule(ConnectionType.SUBSCRIBER, name + '/goal', node), type_info, xmlrpc_uri))
            type_info = rostopic.get_topic_type(name + '/cancel')[0]  # message type
            connections.append(Connection(Rule(ConnectionType.SUBSCRIBER, name + '/cancel', node), type_info, xmlrpc_uri))
            type_info = rostopic.get_topic_type(name + '/status')[0]  # message type
            connections.append(Connection(Rule(ConnectionType.PUBLISHER, name + '/status', node), type_info, xmlrpc_uri))
            type_info = rostopic.get_topic_type(name + '/feedback')[0]  # message type
            connections.append(Connection(Rule(ConnectionType.PUBLISHER, name + '/feedback', node), type_info, xmlrpc_uri))
            type_info = rostopic.get_topic_type(name + '/result')[0]  # message type
            connections.append(Connection(Rule(ConnectionType.PUBLISHER, name + '/result', node), type_info, xmlrpc_uri))
        elif type == ConnectionType.ACTION_CLIENT:
            type_info = rostopic.get_topic_type(name + '/goal')[0]  # message type
            connections.append(Connection(Rule(ConnectionType.PUBLISHER, name + '/goal', node), type_info, xmlrpc_uri))
            type_info = rostopic.get_topic_type(name + '/cancel')[0]  # message type
            connections.append(Connection(Rule(ConnectionType.PUBLISHER, name + '/cancel', node), type_info, xmlrpc_uri))
            type_info = rostopic.get_topic_type(name + '/status')[0]  # message type
            connections.append(Connection(Rule(ConnectionType.SUBSCRIBER, name + '/status', node), type_info, xmlrpc_uri))
            type_info = rostopic.get_topic_type(name + '/feedback')[0]  # message type
            connections.append(Connection(Rule(ConnectionType.SUBSCRIBER, name + '/feedback', node), type_info, xmlrpc_uri))
            type_info = rostopic.get_topic_type(name + '/result')[0]  # message type
            connections.append(Connection(Rule(ConnectionType.SUBSCRIBER, name + '/result', node), type_info, xmlrpc_uri))
        return connections

    def get_ros_ip(self):
        o = urlparse.urlparse(rosgraph.get_master_uri())
        if o.hostname == 'localhost':
            ros_ip = ''
            try:
                ros_ip = os.environ['ROS_IP']
            except Exception:
                try:
                    # often people use this one instead
                    ros_ip = os.environ['ROS_HOSTNAME']
                except Exception:
                    # should probably check other means here - e.g. first of the system ipconfig
                    rospy.logwarn("Gateway: no valid ip found for this host, just setting 'localhost'")
                    return 'localhost'
            return ros_ip
        else:
            return o.hostname

    def _isTopicNodeInList(self, topic, node, topic_node_list):
        # check if cancel available
        available = False
        for candidate in topic_node_list:
            if candidate[0] == topic and node in candidate[1]:
                available = True
                break
        return available

    def _get_actions(self, pubs, subs):
        '''
          Return actions and pruned publisher, subscriber lists.

          @param publishers
          @type list of publishers in the form returned by rosgraph.Master.getSystemState
          @param subscribers
          @type list of subscribers in the form returned by rosgraph.Master.getSystemState
          @return list of actions, pruned_publishers, pruned_subscribers
          @rtype [base_topic, [nodes]], as param type, as param type
        '''

        actions = []
        for goal_candidate in pubs:
            if re.search('goal$', goal_candidate[0]):
                # goal found, extract base topic
                base_topic = re.sub('\/goal$', '', goal_candidate[0])
                nodes = goal_candidate[1]
                action_nodes = []

                # there may be multiple nodes -- for each node search for the other topics
                for node in nodes:
                    is_action = True
                    is_action &= self._isTopicNodeInList(base_topic + '/cancel', node, pubs)
                    is_action &= self._isTopicNodeInList(base_topic + '/status', node, subs)
                    is_action &= self._isTopicNodeInList(base_topic + '/feedback', node, subs)
                    is_action &= self._isTopicNodeInList(base_topic + '/result', node, subs)

                    if is_action:
                        action_nodes.append(node)

                if len(action_nodes) != 0:
                    # yay! an action has been found
                    actions.append([base_topic, action_nodes])
                    # remove action entries from publishers/subscribers
                    for connection in pubs:
                        if connection[0] in [base_topic + '/goal', base_topic + '/cancel']:
                            connection[1].remove(node)
                    for connection in subs:
                        if connection[0] in [base_topic + '/status', base_topic + '/feedback', base_topic + '/result']:
                            connection[1].remove(node)
        pubs[:] = [connection for connection in pubs if len(connection[1]) != 0]
        subs[:] = [connection for connection in subs if len(connection[1]) != 0]
        return actions, pubs, subs

    def getActionServers(self, publishers, subscribers):
        '''
          Return action servers and pruned publisher, subscriber lists.

          @param publishers
          @type list of publishers in the form returned by rosgraph.Master.getSystemState
          @param subscribers
          @type list of subscribers in the form returned by rosgraph.Master.getSystemState
          @return list of actions, pruned_publishers, pruned_subscribers
          @rtype [base_topic, [nodes]], as param type, as param type
        '''
        actions, subs, pubs = self._get_actions(subscribers, publishers)
        return actions, pubs, subs

    def getActionClients(self, publishers, subscribers):
        '''
          Return action clients and pruned publisher, subscriber lists.

          @param publishers
          @type list of publishers in the form returned by rosgraph.Master.getSystemState
          @param subscribers
          @type list of subscribers in the form returned by rosgraph.Master.getSystemState
          @return list of actions, pruned_publishers, pruned_subscribers
          @rtype [base_topic, [nodes]], as param type, as param type
        '''
        actions, pubs, subs = self._get_actions(publishers, subscribers)
        return actions, pubs, subs

    def getConnectionsFromPubSubList(self, list, type):
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
                rule = Rule(type, topic_name, node)
                connection = Connection(rule, topic_type, node_uri)
                connections.append(connection)
        return connections

    def getConnectionsFromActionList(self, list, type):
        connections = []
        for action in list:
            action_name = action[0]
            goal_topic = action_name + '/goal'
            goal_topic_type = rostopic.get_topic_type(goal_topic)
            topic_type = re.sub('ActionGoal$', '', goal_topic_type[0])  #Base type for action
            nodes = action[1]
            for node in nodes:
                try:
                    node_uri = self.lookupNode(node)
                except:
                    continue
                rule = Rule(type, action_name, node)
                connection = Connection(rule, topic_type, node_uri)
                connections.append(connection)
        return connections

    def getConnectionsFromServiceList(self, list, type):
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

    def _get_anonymous_node_name(self, topic):
        t = topic[1:len(topic)]
        name = roslib.names.anonymous_name(t)
        return name

    ##########################################################################
    # Master utility methods for scripts
    ##########################################################################

    def find_gateway_namespace(self):
        '''
          Assists a script to find the (hopefully) unique gateway namespace.
          Note that unique is a necessary condition, there should only be one
          gateway per ros system.

          @return Namespace of the gateway node.
          @rtype string
        '''
        unused_publishers, unused_subscribers, services = self.getSystemState()
        for service in services:
            service_name = service[0]  # second part is the node name
            if re.search(r'remote_gateway_info', service_name):
                if service_name == '/remote_gateway_info':
                    return "/"
                else:
                    return re.sub(r'/remote_gateway_info', '', service_name)
        return None
