#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway_graph/LICENSE
#
'''
  This code does not effect the runtime of gateways at all - it is used for
  debugging and monitoring purposes only.
'''
##############################################################################
# Imports
##############################################################################

import roslib
roslib.load_manifest('rocon_gateway')
import rospy
import gateway_msgs.srv
from master_api import LocalMaster
import rosgraph
from rosgraph.impl.graph import Edge, EdgeList

##############################################################################
# Graph
##############################################################################


class Graph(object):
    '''
    Utility class for polling statistics from a running gateway-hub network.
    '''

    def __init__(self):
        '''
        Creates the polling topics necessary for updating statistics
        about the running gateway-hub network.
        '''
        self._last_update = 0
        self._gateway_namespace = None
        self._local_gateway = None
        self._remote_gateways = None
        self.gateway_nodes = []  # Gateway nodes
        self.connection_nodes = []  # Topic nodes
        self.nn_edges = []  # Gateway-Gateway edges
        self.nt_edges = []  # Gateway-Topic edges
        self.nt_all_edges = []  # All unconnected node-topics or topic-nodes

        # Rubbish to clear out once rocon_gateway_graph is integrated
        self.bad_nodes = []

        if self._resolve_gateway_namespace():
            self.configure()

    def configure(self):
        self._gateway_info = rospy.ServiceProxy(self.gateway_namespace + '/gateway_info', gateway_msgs.srv.GatewayInfo)
        self._remote_gateway_info = rospy.ServiceProxy(self.gateway_namespace + '/remote_gateway_info', gateway_msgs.srv.RemoteGatewayInfo)

    def update(self):
        if not self._resolve_gateway_namespace():
            return
        req = gateway_msgs.srv.GatewayInfoRequest()
        self._local_gateway = self._gateway_info(req)
        req = gateway_msgs.srv.RemoteGatewayInfoRequest()
        req.gateways = []
        self._remote_gateways = self._remote_gateway_info(req).gateways
        self._last_update = rospy.get_rostime()
        # Gateways
        self.gateway_nodes.append(self._local_gateway.name)
        self.gateway_nodes.extend([remote_gateway.name for remote_gateway in self._remote_gateways])
        # Connected Gateways
        self.nn_edges = EdgeList()
        self.nt_edges = EdgeList()
        self.nt_all_edges = EdgeList()
        # Check local gateway
        for remote_rule in self._local_gateway.flipped_connections:
            self.nn_edges.add(Edge(self._local_gateway.name, remote_rule.gateway))
            # this adds a bloody magic space, to help disambiguate node names from topic names
            topic_identifier = rosgraph.impl.graph.topic_node(remote_rule.rule.name + '-' + remote_rule.rule.type)
            self.connection_nodes.append(topic_identifier)
            self.nt_edges.add(Edge(self._local_gateway.name, topic_identifier))
            self.nt_edges.add(Edge(topic_identifier, remote_rule.gateway))
            self.nt_all_edges.add(Edge(self._local_gateway.name, topic_identifier))
            self.nt_all_edges.add(Edge(topic_identifier, remote_rule.gateway))
        # Check remote gateways
        for remote_gateway in self._remote_gateways:
            for remote_rule in remote_gateway.flipped_interface:
                self.nn_edges.add(Edge(remote_gateway.name, remote_rule.gateway))

    def _resolve_gateway_namespace(self):
        '''
          Checks if the gateway namespace was found and if not
          attempts to resolve it.
        '''
        if self._gateway_namespace:
            return
        master = LocalMaster()
        self.gateway_namespace = master.findGatewayNamespace()
        if not self.gateway_namespace:
            rospy.logerr("Gateway Graph: could not find a local gateway - did you start it?")
        return self.gateway_namespace
