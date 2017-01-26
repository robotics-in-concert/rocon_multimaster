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
from contextlib import contextmanager

from rosmaster.util import xmlrpcapi

import rocon_python_comms

try:
    import urllib.parse as urlparse  # Python 3.x
except ImportError:
    import urlparse
import re
import threading

import rospy
import rosgraph
import rostopic
import rosservice
import roslib.names
import gateway_msgs.msg as gateway_msgs
import rocon_gateway_utils

from . import utils, GatewayError


class LocalMaster(rosgraph.Master):

    '''
      Representing a ros master (local ros master). Just contains a
      few utility methods for retrieving master related information as well
      as handles for registering and unregistering rules that have
      been pulled or flipped in from another gateway.
    '''

    def __init__(self, connection_cache_timeout=None):
        rosgraph.Master.__init__(self, rospy.get_name())

        timeout = connection_cache_timeout or rospy.Time(30)

        self.connections_lock = threading.Lock()
        self.connections = utils.create_empty_connection_type_dictionary(set)
        # in case this class is used directly (script call) we need to find the connection cache

        connection_cache_namespace = rocon_gateway_utils.resolve_connection_cache(timeout)
        if not connection_cache_namespace.endswith('/'):
            connection_cache_namespace += '/'

        self.connection_cache = rocon_python_comms.ConnectionCacheProxy(
            list_sub=connection_cache_namespace + 'connection_cache/list',
            handle_actions=True,
            user_callback=self._connection_cache_proxy_cb,
            diff_opt=True,
            diff_sub=connection_cache_namespace + 'connection_cache/diff',
            list_wait_timeout=30  # be generous in timeout in case system is slow to start
        )
        self.get_system_state = self.connection_cache.getSystemState

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
        if registration.connection.rule.type == rocon_python_comms.PUBLISHER:
            try:
                node_master.registerPublisher(
                    registration.connection.rule.name,
                    registration.connection.type_info,
                    registration.connection.xmlrpc_uri)
                return registration
            except (socket.error, socket.gaierror) as e:
                rospy.logerr("Gateway : got socket error trying to register a publisher on the local master [%s][%s]" % (
                    registration.connection.rule.name, str(e)))
                return None
            except rosgraph.masterapi.Error as e:
                rospy.logerr("Gateway : got error trying to register a publisher on the local master [%s][%s]" % (
                    registration.connection.rule.name, str(e)))
                return None
            except rosgraph.masterapi.Failure as e:
                rospy.logerr("Gateway : failed to register a publisher on the local master [%s][%s]" % (
                    registration.connection.rule.name, str(e)))
                return None
        elif registration.connection.rule.type == rocon_python_comms.SUBSCRIBER:
            try:
                self._register_subscriber(
                    node_master,
                    registration.connection.rule.name,
                    registration.connection.type_info,
                    registration.connection.xmlrpc_uri)
                return registration
            except (socket.error, socket.gaierror) as e:
                rospy.logerr("Gateway : got socket error trying to register a subscriber on the local master [%s][%s]" % (
                    registration.connection.rule.name, str(e)))
                return None
            except rosgraph.masterapi.Error as e:
                rospy.logerr("Gateway : got error trying to register a subscriber on the local master [%s][%s]" % (
                    registration.connection.rule.name, str(e)))
                return None
            except rosgraph.masterapi.Failure as e:
                rospy.logerr("Gateway : failed to register a subscriber on the local master [%s][%s]" % (
                    registration.connection.rule.name, str(e)))
                return None
        elif registration.connection.rule.type == rocon_python_comms.SERVICE:
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
                    return registration
                except (socket.error, socket.gaierror) as e:
                    rospy.logerr("Gateway : got socket error trying to register a service on the local master [%s][%s]" % (
                        registration.connection.rule.name, str(e)))
                    return None
                except rosgraph.masterapi.Error as e:
                    rospy.logerr("Gateway : got error trying to register a service on the local master [%s][%s]" % (
                        registration.connection.rule.name, str(e)))
                    return None
                except rosgraph.masterapi.Failure as e:
                    rospy.logerr("Gateway : failed to register a service on the local master [%s][%s]" % (
                        registration.connection.rule.name, str(e)))
                    return None
        elif registration.connection.rule.type == rocon_python_comms.ACTION_SERVER:
            try:
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
            except (socket.error, socket.gaierror) as e:
                rospy.logerr("Gateway : got socket error trying to register an action server on the local master [%s][%s]" % (
                    registration.connection.rule.name, str(e)))
                return None
            except rosgraph.masterapi.Error as e:
                rospy.logerr("Gateway : got error trying to register an action server on the local master [%s][%s]" % (
                    registration.connection.rule.name, str(e)))
                return None
            except rosgraph.masterapi.Failure as e:
                rospy.logerr("Gateway : failed to register an action server on the local master [%s][%s]" % (
                    registration.connection.rule.name, str(e)))
                return None
        elif registration.connection.rule.type == rocon_python_comms.ACTION_CLIENT:
            try:
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
            except (socket.error, socket.gaierror) as e:
                rospy.logerr("Gateway : got socket error trying to register an action server on the local master [%s][%s]" % (
                        registration.connection.rule.name, str(e)))
                return None
            except rosgraph.masterapi.Error as e:
                rospy.logerr("Gateway : got error trying to register an action server on the local master [%s][%s]" % (
                    registration.connection.rule.name, str(e)))
                return None
            except rosgraph.masterapi.Failure as e:
                rospy.logerr("Gateway : failed to register an action server on the local master [%s][%s]" % (
                    registration.connection.rule.name, str(e)))
                return None
        else:
            rospy.logerr("Gateway : tried to register unknown rule type [%s]" % (
                registration.connection.rule.type))
            return None

    def unregister(self, registration):
        '''
          Unregisters a rule with the local master.

          @param registration : registration details for an existing gateway registered rule
          @type utils.Registration
        '''
        node_master = rosgraph.Master(registration.local_node)
        rospy.logdebug("Gateway : unregistering local node [%s] for [%s]" % (registration.local_node, registration))
        if registration.connection.rule.type == rocon_python_comms.PUBLISHER:
            try:
                node_master.unregisterPublisher(registration.connection.rule.name, registration.connection.xmlrpc_uri)
            except (socket.error, socket.gaierror) as e:
                rospy.logerr("Gateway : got socket error trying to unregister a publisher on the local master [%s][%s]" % (
                    registration.connection.rule.name, str(e)))
            except rosgraph.masterapi.Error as e:
                rospy.logerr("Gateway : got error trying to unregister a publisher on the local master [%s][%s]" % (
                    registration.connection.rule.name, str(e)))
            except rosgraph.masterapi.Failure as e:
                rospy.logerr("Gateway : failed to unregister a publisher on the local master [%s][%s]" % (
                    registration.connection.rule.name, str(e)))
        elif registration.connection.rule.type == rocon_python_comms.SUBSCRIBER:
            try:
                node_master.unregisterSubscriber(registration.connection.rule.name, registration.connection.xmlrpc_uri)
            except (socket.error, socket.gaierror) as e:
                rospy.logerr("Gateway : got socket error trying to unregister a subscriber on the local master [%s][%s]" % (
                    registration.connection.rule.name, str(e)))
            except rosgraph.masterapi.Error as e:
                rospy.logerr("Gateway : got error trying to unregister a subscriber on the local master [%s][%s]" % (
                    registration.connection.rule.name, str(e)))
            except rosgraph.masterapi.Failure as e:
                rospy.logerr("Gateway : failed to unregister a subscriber on the local master [%s][%s]" % (
                    registration.connection.rule.name, str(e)))
        elif registration.connection.rule.type == rocon_python_comms.SERVICE:
            try:
                node_master.unregisterService(registration.connection.rule.name, registration.connection.type_info)
            except (socket.error, socket.gaierror) as e:
                rospy.logerr("Gateway : got socket error trying to unregister a service on the local master [%s][%s]" % (
                    registration.connection.rule.name, str(e)))
            except rosgraph.masterapi.Error as e:
                rospy.logerr("Gateway : got error trying to unregister a service on the local master [%s][%s]" % (
                    registration.connection.rule.name, str(e)))
            except rosgraph.masterapi.Failure as e:
                rospy.logerr("Gateway : failed to unregister a service on the local master [%s][%s]" % (
                    registration.connection.rule.name, str(e)))
        elif registration.connection.rule.type == rocon_python_comms.ACTION_SERVER:
            try:
                node_master.unregisterSubscriber(
                    registration.connection.rule.name + "/goal", registration.connection.xmlrpc_uri)
                node_master.unregisterSubscriber(
                    registration.connection.rule.name + "/cancel", registration.connection.xmlrpc_uri)
                node_master.unregisterPublisher(
                    registration.connection.rule.name + "/status", registration.connection.xmlrpc_uri)
                node_master.unregisterPublisher(
                    registration.connection.rule.name + "/feedback", registration.connection.xmlrpc_uri)
                node_master.unregisterPublisher(
                    registration.connection.rule.name + "/result", registration.connection.xmlrpc_uri)
            except (socket.error, socket.gaierror) as e:
                rospy.logerr("Gateway : got socket error trying to unregister an action server on the local master [%s][%s]" % (
                    registration.connection.rule.name, str(e)))
            except rosgraph.masterapi.Error as e:
                rospy.logerr("Gateway : got error trying to unregister an action server on the local master [%s][%s]" % (
                    registration.connection.rule.name, str(e)))
            except rosgraph.masterapi.Failure as e:
                rospy.logerr("Gateway : failed to unregister an action server on the local master [%s][%s]" % (
                    registration.connection.rule.name, str(e)))
        elif registration.connection.rule.type == rocon_python_comms.ACTION_CLIENT:
            try:
                node_master.unregisterPublisher(
                    registration.connection.rule.name + "/goal", registration.connection.xmlrpc_uri)
                node_master.unregisterPublisher(
                    registration.connection.rule.name + "/cancel", registration.connection.xmlrpc_uri)
                node_master.unregisterSubscriber(
                    registration.connection.rule.name + "/status", registration.connection.xmlrpc_uri)
                node_master.unregisterSubscriber(
                    registration.connection.rule.name + "/feedback", registration.connection.xmlrpc_uri)
                node_master.unregisterSubscriber(
                    registration.connection.rule.name + "/result", registration.connection.xmlrpc_uri)
            except (socket.error, socket.gaierror) as e:
                rospy.logerr("Gateway : got socket error trying to unregister an action client on the local master [%s][%s]" % (
                    registration.connection.rule.name, str(e)))
            except rosgraph.masterapi.Error as e:
                rospy.logerr("Gateway : got error trying to unregister an action client on the local master [%s][%s]" % (
                    registration.connection.rule.name, str(e)))
            except rosgraph.masterapi.Failure as e:
                rospy.logerr("Gateway : failed to unregister an action client on the local master [%s][%s]" % (
                    registration.connection.rule.name, str(e)))

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
        # Be nice to the subscriber, inform it that is should refresh it's publisher list.
        try:
            rospy.loginfo(
                "resetting publishers for this node's subscriber [%s][%s][%s]" % (name, xmlrpc_uri, pub_uri_list))
            # this publisherUpdate will overwrite any other publisher currently known by the subscriber
            xmlrpcapi(xmlrpc_uri).publisherUpdate('/master', name, pub_uri_list)

        except (socket.error, socket.gaierror) as v:
            errorcode = v[0]
            if errorcode != errno.ECONNREFUSED:
                rospy.logerr(
                    "Gateway : error registering subscriber " +
                    "(is ROS_MASTER_URI and ROS_HOSTNAME or ROS_IP correctly set?)")
                rospy.logerr("Gateway : errorcode [%s] xmlrpc_uri [%s]" % (str(errorcode), xmlrpc_uri))
                raise  # better handling here would be ideal
            else:
                pass
                # subscriber stopped on the other side, so don't worry about telling it to 'refresh' its publishers
        except xmlrpclib.Fault as e:
            # as above with the socket error, don't worry about telling it to 'refresh' its publishers
            rospy.logerr("Gateway : serious fault while communicating with a subscriber - its xmlrpc server was around but in a bad state [%s]" % str(e))
            rospy.logerr("Gateway : if this happened, add to the collected information gathered at https://github.com/robotics-in-concert/rocon_multimaster/issues/304")

    ##########################################################################
    # Master utility methods
    ##########################################################################

    def generate_connection_details(self, connection_type, name, node):
        """
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
        """
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
        if connection_type == rocon_python_comms.PUBLISHER or connection_type == rocon_python_comms.SUBSCRIBER:
            type_info = rostopic.get_topic_type(name)[0]  # message type
            if type_info is not None:
                connections.append(utils.Connection(gateway_msgs.Rule(connection_type, name, node), type_info, type_info, xmlrpc_uri))
            else:
                rospy.logwarn('Gateway : [%s] does not have type_info. Cannot flip' % name)
        elif connection_type == rocon_python_comms.SERVICE:
            type_info = rosservice.get_service_uri(name)
            type_msg = rosservice.get_service_type(name)
            if type_info is not None:
                connections.append(utils.Connection(gateway_msgs.Rule(connection_type, name, node), type_msg, type_info, xmlrpc_uri))
        elif connection_type == rocon_python_comms.ACTION_SERVER:
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
                connections.append(utils.Connection(gateway_msgs.Rule(rocon_python_comms.SUBSCRIBER, name + '/goal', node), goal_type_info, goal_type_info, xmlrpc_uri))
                connections.append(
                    utils.Connection(gateway_msgs.Rule(rocon_python_comms.SUBSCRIBER, name + '/cancel', node), cancel_type_info, cancel_type_info, xmlrpc_uri))
                connections.append(
                    utils.Connection(gateway_msgs.Rule(rocon_python_comms.PUBLISHER, name + '/status', node), status_type_info, status_type_info, xmlrpc_uri))
                connections.append(
                    utils.Connection(gateway_msgs.Rule(rocon_python_comms.PUBLISHER, name + '/feedback', node), feedback_type_info, feedback_type_info, xmlrpc_uri))
                connections.append(
                    utils.Connection(gateway_msgs.Rule(rocon_python_comms.PUBLISHER, name + '/result', node), result_type_info, result_type_info, xmlrpc_uri))
        elif connection_type == rocon_python_comms.ACTION_CLIENT:
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
                connections.append(utils.Connection(gateway_msgs.Rule(rocon_python_comms.PUBLISHER, name + '/goal', node), goal_type_info, goal_type_info, xmlrpc_uri))
                connections.append(
                    utils.Connection(gateway_msgs.Rule(rocon_python_comms.PUBLISHER, name + '/cancel', node), cancel_type_info, cancel_type_info, xmlrpc_uri))
                connections.append(
                    utils.Connection(gateway_msgs.Rule(rocon_python_comms.SUBSCRIBER, name + '/status', node), status_type_info, status_type_info, xmlrpc_uri))
                connections.append(
                    utils.Connection(gateway_msgs.Rule(rocon_python_comms.SUBSCRIBER, name + '/feedback', node), feedback_type_info, feedback_type_info, xmlrpc_uri))
                connections.append(
                    utils.Connection(gateway_msgs.Rule(rocon_python_comms.SUBSCRIBER, name + '/result', node), result_type_info, result_type_info, xmlrpc_uri))
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
        if connection_type == rocon_python_comms.PUBLISHER or connection_type == rocon_python_comms.SUBSCRIBER:
            type_info = rostopic.get_topic_type(name)[0]  # message type
            if type_info is not None:
                connection = utils.Connection(gateway_msgs.Rule(connection_type, name, node), type_info, type_info, xmlrpc_uri)
        elif connection_type == rocon_python_comms.SERVICE:
            type_info = rosservice.get_service_uri(name)
            type_msg = rosservice.get_service_type(name)
            if type_info is not None:
                connection = utils.Connection(gateway_msgs.Rule(connection_type, name, node), type_msg, type_info, xmlrpc_uri)
        elif connection_type == rocon_python_comms.ACTION_SERVER or connection_type == rocon_python_comms.ACTION_CLIENT:
            goal_topic = name + '/goal'
            goal_topic_type = rostopic.get_topic_type(goal_topic)
            type_info = re.sub('ActionGoal$', '', goal_topic_type[0])  # Base type for action
            if type_info is not None:
                connection = utils.Connection(gateway_msgs.Rule(connection_type, name, node), type_info, type_info, xmlrpc_uri)
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
                    rospy.logwarn("Gateway : no ROS_IP/HOSTNAME for this host -> using 'localhost'")
                    return 'localhost'
            return ros_ip
        else:
            return o.hostname

    @staticmethod
    def _is_topic_node_in_list(topic, node, topic_node_list):
        # TODO : there is probably a oneliner equivalent for this
        # check if cancel available
        available = False
        for candidate in topic_node_list:
            if candidate[0] == topic and node in candidate[1]:
                available = True
                break

    @staticmethod
    def _get_anonymous_node_name(topic):
        t = topic[1:len(topic)]
        name = roslib.names.anonymous_name(t)
        return name

    def _connection_cache_proxy_cb(self, system_state, added_system_state, lost_system_state):

        self.connections_lock.acquire()
        # if there was no change but we got a callback,
        # it means it s the first and we need to set the whole list
        if added_system_state is None and lost_system_state is None:
            self.connections[gateway_msgs.ConnectionType.ACTION_SERVER] =\
                utils._get_connections_from_action_chan_dict(
                        system_state.action_servers, gateway_msgs.ConnectionType.ACTION_SERVER
                )
            self.connections[gateway_msgs.ConnectionType.ACTION_CLIENT] =\
                utils._get_connections_from_action_chan_dict(
                        system_state.action_clients, gateway_msgs.ConnectionType.ACTION_CLIENT
                )
            self.connections[gateway_msgs.ConnectionType.PUBLISHER] =\
                utils._get_connections_from_pub_sub_chan_dict(
                        system_state.publishers, gateway_msgs.ConnectionType.PUBLISHER
                )
            self.connections[gateway_msgs.ConnectionType.SUBSCRIBER] =\
                utils._get_connections_from_pub_sub_chan_dict(
                        system_state.subscribers, gateway_msgs.ConnectionType.SUBSCRIBER
                )
            self.connections[gateway_msgs.ConnectionType.SERVICE] =\
                utils._get_connections_from_service_chan_dict(
                        system_state.services, gateway_msgs.ConnectionType.SERVICE
                )
        else:  # we got some diff, we can optimize
            # this is not really needed as is ( since the list we get is always updated )
            # However it is a first step towards an optimized gateway that can work with system state differences
            new_action_servers = utils._get_connections_from_action_chan_dict(
                added_system_state.action_servers, gateway_msgs.ConnectionType.ACTION_SERVER
            )
            self.connections[gateway_msgs.ConnectionType.ACTION_SERVER] |= new_action_servers

            new_action_clients = utils._get_connections_from_action_chan_dict(
                added_system_state.action_clients, gateway_msgs.ConnectionType.ACTION_CLIENT
            )
            self.connections[gateway_msgs.ConnectionType.ACTION_CLIENT] |= new_action_clients

            new_publishers = utils._get_connections_from_pub_sub_chan_dict(
                added_system_state.publishers, gateway_msgs.ConnectionType.PUBLISHER
            )
            self.connections[gateway_msgs.ConnectionType.PUBLISHER] |= new_publishers

            new_subscribers = utils._get_connections_from_pub_sub_chan_dict(
                added_system_state.subscribers, gateway_msgs.ConnectionType.SUBSCRIBER
            )
            self.connections[gateway_msgs.ConnectionType.SUBSCRIBER] |= new_subscribers

            new_services = utils._get_connections_from_service_chan_dict(
                added_system_state.services, gateway_msgs.ConnectionType.SERVICE
            )
            self.connections[gateway_msgs.ConnectionType.SERVICE] |= new_services


            lost_action_servers = utils._get_connections_from_action_chan_dict(
                lost_system_state.action_servers, gateway_msgs.ConnectionType.ACTION_SERVER
            )
            self.connections[gateway_msgs.ConnectionType.ACTION_SERVER] -= lost_action_servers

            lost_action_clients = utils._get_connections_from_action_chan_dict(
                lost_system_state.action_clients, gateway_msgs.ConnectionType.ACTION_CLIENT
            )
            self.connections[gateway_msgs.ConnectionType.ACTION_CLIENT] -= lost_action_clients

            lost_publishers = utils._get_connections_from_pub_sub_chan_dict(
                lost_system_state.publishers, gateway_msgs.ConnectionType.PUBLISHER
            )
            self.connections[gateway_msgs.ConnectionType.PUBLISHER] -= lost_publishers

            lost_subscribers = utils._get_connections_from_pub_sub_chan_dict(
                lost_system_state.subscribers, gateway_msgs.ConnectionType.SUBSCRIBER
            )
            self.connections[gateway_msgs.ConnectionType.SUBSCRIBER] -= lost_subscribers

            lost_services = utils._get_connections_from_service_chan_dict(
                lost_system_state.services, gateway_msgs.ConnectionType.SERVICE
            )
            self.connections[gateway_msgs.ConnectionType.SERVICE] -= lost_services

        self.connections_lock.release()

    @contextmanager
    def get_connection_state(self):
        self.connections_lock.acquire()
        yield self.connections
        self.connections_lock.release()

