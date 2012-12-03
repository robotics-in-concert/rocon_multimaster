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
        self._remote_gateways = self._remote_gateway_info(req)
        self._last_update = rospy.get_rostime()

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
