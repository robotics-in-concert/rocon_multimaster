#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os
import socket
import httplib
import errno
import xmlrpclib
from rosmaster.util import xmlrpcapi
try:
    import urllib.parse as urlparse  # Python 3.x
except ImportError:
    import urlparse
import re

import rospy
import rosgraph
import rostopic
import rosservice
import roslib.names
from gateway_msgs.msg import Rule, ConnectionType

from . import utils

##############################################################################
# Aliases
##############################################################################

# Can't see an easier way to alias or import these
PUBLISHER = ConnectionType.PUBLISHER
SUBSCRIBER = ConnectionType.SUBSCRIBER
SERVICE = ConnectionType.SERVICE
ACTION_SERVER = ConnectionType.ACTION_SERVER
ACTION_CLIENT = ConnectionType.ACTION_CLIENT

##############################################################################
# Master
##############################################################################


class ConnectionCache(object):

    def __init__(self, get_system_state):
        self._get_system_state = get_system_state  # function call to the local master
        self._connections = {}
#        self._system_state = {}
#        self._system_state[PUBLISHER] = []
#        self._system_state[SUBSCRIBER] = []
#        self._system_state[SERVICE] = []
        self._connections = utils.create_empty_connection_type_dictionary()

    def update(self, new_system_state=None):
        '''
          Currently completely regenerating the connections dictionary and then taking
          diffs. Could be faster if we took diffs on the system state instead, but that's
          a bit more awkward since each element has a variable list of nodes that we'd have
          to check against to get good diffs. e.g.
            old_publishers = ['/chatter', ['/talker']]
            new_publishers = ['/chatter', ['/talker', '/babbler']]
        '''
        # init the variables we will return
        new_connections = utils.create_empty_connection_type_dictionary()
        lost_connections = utils.create_empty_connection_type_dictionary()

        if new_system_state is None:
            try:
                publishers, subscribers, services = self._get_system_state()
            except socket.error:
                rospy.logerr("Gateway : couldn't get system state from the master "
                             "[did you set your master uri to a wireless IP that just went down?]")
                return new_connections, lost_connections
        else:
            publishers = new_system_state[PUBLISHER]
            subscribers = new_system_state[SUBSCRIBER]
            services = new_system_state[SERVICE]
        action_servers, publishers, subscribers = self._get_action_servers(publishers, subscribers)
        action_clients, publishers, subscribers = self._get_action_clients(publishers, subscribers)
        connections = utils.create_empty_connection_type_dictionary()
        connections[PUBLISHER] = self._get_connections_from_pub_sub_list(publishers, PUBLISHER)
        connections[SUBSCRIBER] = self._get_connections_from_pub_sub_list(subscribers, SUBSCRIBER)
        connections[SERVICE] = self._get_connections_from_service_list(services, SERVICE)
        connections[ACTION_SERVER] = self._get_connections_from_action_list(action_servers, ACTION_SERVER)
        connections[ACTION_CLIENT] = self._get_connections_from_action_list(action_clients, ACTION_CLIENT)

        # Will probably need to check not just in, but only name, node equal
        for connection_type in utils.connection_types:
            new_connections[connection_type] = utils.difflist(connections[connection_type], self._connections[connection_type])
            lost_connections[connection_type] = utils.difflist(self._connections[connection_type], connections[connection_type])
        self._connections = connections
        return new_connections, lost_connections

    def _is_topic_node_in_list(self, topic, node, topic_node_list):
        # check if cancel available
        available = False
        for candidate in topic_node_list:
            if candidate[0] == topic and node in candidate[1]:
                available = True
                break
        return available

    def _get_connections_from_action_list(self, connection_list, connection_type):
        connections = []
        for action in connection_list:
            action_name = action[0]
            #goal_topic = action_name + '/goal'
            #goal_topic_type = rostopic.get_topic_type(goal_topic)
            # topic_type = re.sub('ActionGoal$', '', goal_topic_type[0])  # Base type for action
            nodes = action[1]
            for node in nodes:
                # try:
                #    node_uri = self.lookupNode(node)
                # except:
                #    continue
                rule = Rule(connection_type, action_name, node)
                connection = utils.Connection(rule, None, None)  # topic_type, node_uri
                connections.append(connection)
        return connections

    def _get_connections_from_service_list(self, connection_list, connection_type):
        connections = []
        for service in connection_list:
            service_name = service[0]
            #service_uri = rosservice.get_service_uri(service_name)
            nodes = service[1]
            for node in nodes:
                # try:
                #    node_uri = self.lookupNode(node)
                # except:
                #    continue
                rule = Rule(connection_type, service_name, node)
                connection = utils.Connection(rule, None, None)  # service_uri, node_uri
                connections.append(connection)
        return connections

    def _get_connections_from_pub_sub_list(self, connection_list, connection_type):
        connections = []
        for topic in connection_list:
            topic_name = topic[0]
            #topic_type = rostopic.get_topic_type(topic_name)
            #topic_type = topic_type[0]
            nodes = topic[1]
            for node in nodes:
                # try:
                    #node_uri = self.lookupNode(node)
                # except:
                #    continue
                rule = Rule(connection_type, topic_name, node)
                connection = utils.Connection(rule, None, None)  # topic_type, node_uri
                connections.append(connection)
        return connections

    def _get_actions(self, pubs, subs):
        '''
          Return actions and pruned publisher, subscriber lists.

          @param publishers
          @type list of publishers in the form returned by rosgraph.Master.get_system_state
          @param subscribers
          @type list of subscribers in the form returned by rosgraph.Master.get_system_state
          @return list of actions, pruned_publishers, pruned_subscribers
          @rtype [base_topic, [nodes]], as param type, as param type
        '''

        actions = []
        for goal_candidate in pubs:
            if re.search('\/goal$', goal_candidate[0]):
                # goal found, extract base topic
                base_topic = re.sub('\/goal$', '', goal_candidate[0])
                nodes = goal_candidate[1]
                action_nodes = []

                # there may be multiple nodes -- for each node search for the other topics
                for node in nodes:
                    is_action = True
                    is_action &= self._is_topic_node_in_list(base_topic + '/goal', node, pubs)
                    is_action &= self._is_topic_node_in_list(base_topic + '/cancel', node, pubs)
                    is_action &= self._is_topic_node_in_list(base_topic + '/status', node, subs)
                    is_action &= self._is_topic_node_in_list(base_topic + '/feedback', node, subs)
                    is_action &= self._is_topic_node_in_list(base_topic + '/result', node, subs)

                    if is_action:
                        action_nodes.append(node)

                if len(action_nodes) != 0:
                    # yay! an action has been found
                    actions.append([base_topic, action_nodes])
                    # remove action entries from publishers/subscribers
                    for connection in pubs:
                        if connection[0] in [base_topic + '/goal', base_topic + '/cancel']:
                            for node in action_nodes:
                                try:
                                    connection[1].remove(node)
                                except ValueError:
                                    rospy.logerr(
                                        "Gateway : couldn't remove an action publisher " +
                                        "from the master connections list [%s][%s]" %
                                        (connection[0], node))
                    for connection in subs:
                        if connection[0] in [base_topic + '/status', base_topic + '/feedback', base_topic + '/result']:
                            for node in action_nodes:
                                try:
                                    connection[1].remove(node)
                                except ValueError:
                                    rospy.logerr(
                                        "Gateway : couldn't remove an action subscriber " +
                                        "from the master connections list [%s][%s]" %
                                        (connection[0], node))
        pubs[:] = [connection for connection in pubs if len(connection[1]) != 0]
        subs[:] = [connection for connection in subs if len(connection[1]) != 0]
        return actions, pubs, subs

    def _get_action_servers(self, publishers, subscribers):
        '''
          Return action servers and pruned publisher, subscriber lists.

          @param publishers
          @type list of publishers in the form returned by rosgraph.Master.get_system_state
          @param subscribers
          @type list of subscribers in the form returned by rosgraph.Master.get_system_state
          @return list of actions, pruned_publishers, pruned_subscribers
          @rtype [base_topic, [nodes]], as param type, as param type
        '''
        actions, subs, pubs = self._get_actions(subscribers, publishers)
        return actions, pubs, subs

    def _get_action_clients(self, publishers, subscribers):
        '''
          Return action clients and pruned publisher, subscriber lists.

          @param publishers
          @type list of publishers in the form returned by rosgraph.Master.get_system_state
          @param subscribers
          @type list of subscribers in the form returned by rosgraph.Master.get_system_state
          @return list of actions, pruned_publishers, pruned_subscribers
          @rtype [base_topic, [nodes]], as param type, as param type
        '''
        actions, pubs, subs = self._get_actions(publishers, subscribers)
        return actions, pubs, subs


        # generate diffs
#        new = {}
#        lost = {}
#        new[PUBLISHER] = utils.difflist(publishers, self._system_state[PUBLISHER])
#        lost[PUBLISHER] = utils.difflist(self._system_state[PUBLISHER], publishers)
#        self._is_topic_node_in_list(base_topic + '/goal', node, pubs)
#        new[SUBSCRIBER] = utils.difflist(subscribers, self._system_state[SUBSCRIBER])
#        lost[SUBSCRIBER] = utils.difflist(self._system_state[SUBSCRIBER], subscribers)
#        new[SERVICE] = utils.difflist(services, self._system_state[SERVICE])
#        lost[SERVICE] = utils.difflist(self._system_state[SERVICE], services)
# cache new system state
#        self._system_state[PUBLISHER] = copy.deepcopy(publishers)
#        self._system_state[SUBSCRIBER] = copy.deepcopy(subscribers)
#        self._system_state[SERVICE] = copy.deepcopy(services)
#
#        print("%s" % new[PUBLISHER])
#        print("%s" % lost[PUBLISHER])
        # generate more diffs
#        new[ACTION_SERVER], new[PUBLISHER], new[SUBSCRIBER] = self.get_action_servers(new[PUBLISHER], new[SUBSCRIBER])
#        new[ACTION_CLIENT], new[PUBLISHER], new[SUBSCRIBER] = self.get_action_clients(new[PUBLISHER], new[SUBSCRIBER])
#        lost[ACTION_SERVER], lost[PUBLISHER], lost[SUBSCRIBER] = \
#            self.get_action_servers(lost[PUBLISHER], lost[SUBSCRIBER])
#        lost[ACTION_CLIENT], lost[PUBLISHER], lost[SUBSCRIBER] = \
#            self.get_action_clients(lost[PUBLISHER], lost[SUBSCRIBER])
#
#        self._connections[PUBLISHER].append(self.get_connections_from_pub_sub(new[PUBLISHER], PUBLISHER))
#        self._connections[SUBSCRIBER].append(self.get_connections_from_pub_sub(new[SUBSCRIBER], SUBSCRIBER))
#        self._connections[PUBLISHER] = list(set(self._connections[connection_type]) - set(lost[connection_type]))
#
#            topic_name = topic[0]
#            topic_type = rostopic.get_topic_type(topic_name)
#            topic_type = topic_type[0]
#            nodes = topic[1]
#            for node in nodes:
#                try:
#                    node_uri = self.lookupNode(node)
#                except:
#                    continue
#                rule = Rule(connection_type, topic_name, node)
#                connection = utils.Connection(rule, topic_type, node_uri)

#    def _update_pub_sub_connections(self, connection_type, new_states, lost_states):
#        for topic in new_states:
#            topic_name = topic[0]
#            nodes = topic[1]
#            for node in nodes:
#                rule = Rule(connection_type, topic_name, node)
# connection = utils.Connection(rule, None, None)  # topic_type, node_uri)
#                self._connections[connection_type].append(connection)
#        lost_connections = []
#        for topic in lost_states:
#            topic_name = topic[0]
#            nodes = topic[1]
#            for node in nodes:
#                rule = Rule(connection_type, topic_name, node)
# connection = utils.Connection(rule, None, None)  # topic_type, node_uri)
#                lost_connections.append(connection)
# self._connections[connection_type][:] = [connection for connection in self._connections if ]


class LocalMaster(rosgraph.Master):

    '''
      Representing a ros master (local ros master). Just contains a
      few utility methods for retrieving master related information as well
      as handles for registering and unregistering rules that have
      been pulled or flipped in from another gateway.
    '''

    def __init__(self):
        rosgraph.Master.__init__(self, rospy.get_name())
        # alias
        self.get_system_state = self.getSystemState
        self._connection_cache = ConnectionCache(self.get_system_state)

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
        # rograph.Master doesn't care whether the node is prefixed with slash or not, but we use it to
        # compare registrations later in FlippedInterface._is_registration_in_remote_rule()
        registration.local_node = "/" + self._get_anonymous_node_name(registration.connection.rule.node)
        rospy.logdebug("Gateway : registering a new node [%s] for [%s]" % (registration.local_node, registration))

        # Then do we need checkIfIsLocal? Needs lots of parsing time, and the outer class should
        # already have handle that.

        node_master = rosgraph.Master(registration.local_node)
        if registration.connection.rule.type == PUBLISHER:
            node_master.registerPublisher(
                registration.connection.rule.name,
                registration.connection.type_info,
                registration.connection.xmlrpc_uri)
            return registration
        elif registration.connection.rule.type == SUBSCRIBER:
            self._register_subscriber(
                node_master,
                registration.connection.rule.name,
                registration.connection.type_info,
                registration.connection.xmlrpc_uri)
            return registration
        elif registration.connection.rule.type == SERVICE:
            node_name = rosservice.get_service_node(registration.connection.rule.name)
            if node_name is not None:
                rospy.logwarn(
                    "Gateway : tried to register a service that is already locally available, aborting [%s][%s]" %
                    (registration.connection.rule.name, node_name))
                return None
            else:
                if registration.connection.rule.name is None:
                    rospy.logerr(
                        "Gateway : tried to register a service with name set to None [%s, %s, %s]" %
                        (registration.connection.rule.name,
                         registration.connection.type_info,
                         registration.connection.xmlrpc_uri))
                    return None
                if registration.connection.type_info is None:
                    rospy.logerr(
                        "Gateway : tried to register a service with type_info set to None [%s, %s, %s]" %
                        (registration.connection.rule.name,
                         registration.connection.type_info,
                         registration.connection.xmlrpc_uri))
                    return None
                if registration.connection.xmlrpc_uri is None:
                    rospy.logerr(
                        "Gateway : tried to register a service with xmlrpc_uri set to None [%s, %s, %s]" %
                        (registration.connection.rule.name,
                         registration.connection.type_info,
                         registration.connection.xmlrpc_uri))
                    return None
                try:
                    node_master.registerService(
                        registration.connection.rule.name,
                        registration.connection.type_info,
                        registration.connection.xmlrpc_uri)
                except rosgraph.masterapi.Error as e:
                    rospy.logerr("Gateway : got error trying to register a service on the local master [%s][%s]" % (
                        registration.connection.rule.name, str(e)))
                    return None
                except rosgraph.masterapi.Failure as e:
                    rospy.logerr("Gateway : failed to register a service on the local master [%s][%s]" % (
                        registration.connection.rule.name, str(e)))
                    return None
                return registration
        elif registration.connection.rule.type == ACTION_SERVER:
            # Need to update these with self._register_subscriber
            self._register_subscriber(
                node_master,
                registration.connection.rule.name +
                "/goal",
                registration.connection.type_info +
                "ActionGoal",
                registration.connection.xmlrpc_uri)
            self._register_subscriber(
                node_master,
                registration.connection.rule.name +
                "/cancel",
                "actionlib_msgs/GoalID",
                registration.connection.xmlrpc_uri)
            node_master.registerPublisher(
                registration.connection.rule.name +
                "/status",
                "actionlib_msgs/GoalStatusArray",
                registration.connection.xmlrpc_uri)
            node_master.registerPublisher(
                registration.connection.rule.name +
                "/feedback",
                registration.connection.type_info +
                "ActionFeedback",
                registration.connection.xmlrpc_uri)
            node_master.registerPublisher(
                registration.connection.rule.name +
                "/result",
                registration.connection.type_info +
                "ActionResult",
                registration.connection.xmlrpc_uri)
            return registration
        elif registration.connection.rule.type == ACTION_CLIENT:
            node_master.registerPublisher(
                registration.connection.rule.name +
                "/goal",
                registration.connection.type_info +
                "ActionGoal",
                registration.connection.xmlrpc_uri)
            node_master.registerPublisher(
                registration.connection.rule.name +
                "/cancel",
                "actionlib_msgs/GoalID",
                registration.connection.xmlrpc_uri)
            self._register_subscriber(node_master, registration.connection.rule.name +
                                      "/status", "actionlib_msgs/GoalStatusArray", registration.connection.xmlrpc_uri)
            self._register_subscriber(
                node_master,
                registration.connection.rule.name +
                "/feedback",
                registration.connection.type_info +
                "ActionFeedback",
                registration.connection.xmlrpc_uri)
            self._register_subscriber(
                node_master,
                registration.connection.rule.name +
                "/result",
                registration.connection.type_info +
                "ActionResult",
                registration.connection.xmlrpc_uri)
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
        if registration.connection.rule.type == PUBLISHER:
            node_master.unregisterPublisher(registration.connection.rule.name, registration.connection.xmlrpc_uri)
        elif registration.connection.rule.type == SUBSCRIBER:
            self._unregister_subscriber(
                node_master, registration.connection.xmlrpc_uri, registration.connection.rule.name)
        elif registration.connection.rule.type == SERVICE:
            node_master.unregisterService(registration.connection.rule.name, registration.connection.type_info)
        elif registration.connection.rule.type == ACTION_SERVER:
            self._unregister_subscriber(
                node_master, registration.connection.xmlrpc_uri, registration.connection.rule.name + "/goal")
            self._unregister_subscriber(
                node_master, registration.connection.xmlrpc_uri, registration.connection.rule.name + "/cancel")
            node_master.unregisterPublisher(
                registration.connection.rule.name + "/status", registration.connection.xmlrpc_uri)
            node_master.unregisterPublisher(
                registration.connection.rule.name + "/feedback", registration.connection.xmlrpc_uri)
            node_master.unregisterPublisher(
                registration.connection.rule.name + "/result", registration.connection.xmlrpc_uri)
        elif registration.connection.rule.type == ACTION_CLIENT:
            node_master.unregisterPublisher(
                registration.connection.rule.name + "/goal", registration.connection.xmlrpc_uri)
            node_master.unregisterPublisher(
                registration.connection.rule.name + "/cancel", registration.connection.xmlrpc_uri)
            self._unregister_subscriber(
                node_master, registration.connection.xmlrpc_uri, registration.connection.rule.name + "/status")
            self._unregister_subscriber(
                node_master, registration.connection.xmlrpc_uri, registration.connection.rule.name + "/feedback")
            self._unregister_subscriber(
                node_master, registration.connection.xmlrpc_uri, registration.connection.rule.name + "/result")

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
        except socket.error as v:
            errorcode = v[0]
            if errorcode != errno.ECONNREFUSED:
                rospy.logerr(
                    "Gateway : error registering subscriber " +
                    "(is ROS_MASTER_URI and ROS_HOSTNAME or ROS_IP correctly set?)")
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
        except socket.error as v:
            errorcode = v[0]
            if errorcode != errno.ECONNREFUSED:
                raise  # better handling here would be ideal
            else:
                pass  # subscriber stopped on the other side, don't worry about it
        except xmlrpclib.Fault:
            # This occurs when the subscriber has gone down and unflipped.
            # For us this is not an error since we were only informing
            # the subscriber of an updated publisher state...which
            # it no longer needs!
            pass
        except httplib.CannotSendRequest:
            # This occurs if the master has shut down, just ignore this gracefully.
            # I'm not aware that it's important to catch this at any othe time.
            pass
        node_master.unregisterSubscriber(name, xmlrpc_uri)

    ##########################################################################
    # Master utility methods
    ##########################################################################

    def generate_connection_details(self, connection_type, name, node):
        '''
        Creates all the extra details to create a connection object from a
        rule.

        @param connection_type : the connection type (one of gateway_msgs.msg.ConnectionType)
        @type string
        @param name : the name of the connection
        @type string
        @param node : the master node name it comes from
        @param string

        @return the utils.Connection object complete with type_info and xmlrpc_uri
        @type utils.Connection
        '''
        # Very important here to check for the results of xmlrpc_uri and especially topic_type
        #     https://github.com/robotics-in-concert/rocon_multimaster/issues/173
        # In the watcher thread, we get the local connection index (whereby the arguments of this function
        # come from) via master.get_connection_state. That means there is a small amount of time from
        # getting the topic name, to checking for hte xmlrpc_uri and especially topic_type here in which
        # the topic could have disappeared. When this happens, it returns None.
        connections = []
        xmlrpc_uri = node.split(",")[1]
        node = node.split(",")[0]

        if xmlrpc_uri is None:
            return connections
        if connection_type == PUBLISHER or connection_type == SUBSCRIBER:
            type_info = rostopic.get_topic_type(name)[0]  # message type
            if type_info is not None:
                connections.append(utils.Connection(Rule(connection_type, name, node), type_info, xmlrpc_uri))
            else:
                rospy.logwarn('Gateway : [%s] does not have type_info. Cannot flip' % name)
        elif connection_type == SERVICE:
            type_info = rosservice.get_service_uri(name)
            if type_info is not None:
                connections.append(utils.Connection(Rule(connection_type, name, node), type_info, xmlrpc_uri))
        elif connection_type == ACTION_SERVER:
            goal_type_info = rostopic.get_topic_type(name + '/goal')[0]  # message type
            cancel_type_info = rostopic.get_topic_type(name + '/cancel')[0]  # message type
            status_type_info = rostopic.get_topic_type(name + '/status')[0]  # message type
            feedback_type_info = rostopic.get_topic_type(name + '/feedback')[0]  # message type
            result_type_info = rostopic.get_topic_type(name + '/result')[0]  # message type
            if (
                goal_type_info is not None and cancel_type_info is not None and
                status_type_info is not None and feedback_type_info is not None and
                result_type_info is not None
            ):
                connections.append(utils.Connection(Rule(SUBSCRIBER, name + '/goal', node), goal_type_info, xmlrpc_uri))
                connections.append(
                    utils.Connection(Rule(SUBSCRIBER, name + '/cancel', node), cancel_type_info, xmlrpc_uri))
                connections.append(
                    utils.Connection(Rule(PUBLISHER, name + '/status', node), status_type_info, xmlrpc_uri))
                connections.append(
                    utils.Connection(Rule(PUBLISHER, name + '/feedback', node), feedback_type_info, xmlrpc_uri))
                connections.append(
                    utils.Connection(Rule(PUBLISHER, name + '/result', node), result_type_info, xmlrpc_uri))
        elif connection_type == ACTION_CLIENT:
            goal_type_info = rostopic.get_topic_type(name + '/goal')[0]  # message type
            cancel_type_info = rostopic.get_topic_type(name + '/cancel')[0]  # message type
            status_type_info = rostopic.get_topic_type(name + '/status')[0]  # message type
            feedback_type_info = rostopic.get_topic_type(name + '/feedback')[0]  # message type
            result_type_info = rostopic.get_topic_type(name + '/result')[0]  # message type
            if (
                goal_type_info is not None and cancel_type_info is not None and
                status_type_info is not None and feedback_type_info is not None and
                result_type_info is not None
            ):
                connections.append(utils.Connection(Rule(PUBLISHER, name + '/goal', node), goal_type_info, xmlrpc_uri))
                connections.append(
                    utils.Connection(Rule(PUBLISHER, name + '/cancel', node), cancel_type_info, xmlrpc_uri))
                connections.append(
                    utils.Connection(Rule(SUBSCRIBER, name + '/status', node), status_type_info, xmlrpc_uri))
                connections.append(
                    utils.Connection(Rule(SUBSCRIBER, name + '/feedback', node), feedback_type_info, xmlrpc_uri))
                connections.append(
                    utils.Connection(Rule(SUBSCRIBER, name + '/result', node), result_type_info, xmlrpc_uri))
        return connections

    def generate_advertisement_connection_details(self, connection_type, name, node):
        '''
        Creates all the extra details to create a connection object from an
        advertisement rule. This is a bit different to the previous one - we just need
        the type and single node uri that everything originates from (don't need to generate all
        the pub/sub connections themselves.

        Probably flips could be merged into this sometime, but it'd be a bit gnarly.

        @param connection_type : the connection type (one of gateway_msgs.msg.ConnectionType)
        @type string
        @param name : the name of the connection
        @type string
        @param node : the master node name it comes from
        @param string

        @return the utils.Connection object complete with type_info and xmlrpc_uri
        @type utils.Connection
        '''
        # Very important here to check for the results of xmlrpc_uri and especially topic_type
        #     https://github.com/robotics-in-concert/rocon_multimaster/issues/173
        # In the watcher thread, we get the local connection index (whereby the arguments of this function
        # come from) via master.get_connection_state. That means there is a small amount of time from
        # getting the topic name, to checking for hte xmlrpc_uri and especially topic_type here in which
        # the topic could have disappeared. When this happens, it returns None.
        connection = None
        xmlrpc_uri = self.lookupNode(node)
        if xmlrpc_uri is None:
            return connection
        if connection_type == PUBLISHER or connection_type == SUBSCRIBER:
            type_info = rostopic.get_topic_type(name)[0]  # message type
            if type_info is not None:
                connection = utils.Connection(Rule(connection_type, name, node), type_info, xmlrpc_uri)
        elif connection_type == SERVICE:
            type_info = rosservice.get_service_uri(name)
            if type_info is not None:
                connection = utils.Connection(Rule(connection_type, name, node), type_info, xmlrpc_uri)
        elif connection_type == ACTION_SERVER or connection_type == ACTION_CLIENT:
            goal_topic = name + '/goal'
            goal_topic_type = rostopic.get_topic_type(goal_topic)
            type_info = re.sub('ActionGoal$', '', goal_topic_type[0])  # Base type for action
            if type_info is not None:
                connection = utils.Connection(Rule(connection_type, name, node), type_info, xmlrpc_uri)
        return connection

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
                    rospy.logwarn("Gateway : no valid ip found for this host, just setting 'localhost'")
                    return 'localhost'
            return ros_ip
        else:
            return o.hostname

    def get_connection_state(self):
        unused_new_connections, unused_lost_connections = self._connection_cache.update()
        # This would be more optimal, but we'll have to perturb lots of code
        # return new_connections, lost_connections
        return self._connection_cache._connections

    def _get_anonymous_node_name(self, topic):
        t = topic[1:len(topic)]
        name = roslib.names.anonymous_name(t)
        return name
