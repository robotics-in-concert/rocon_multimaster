#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import rocon_gateway
import uuid
import gateway_msgs.msg as gateway_msgs
import gateway_msgs.srv as gateway_srvs
from urlparse import urlparse

# Local imports
import zeroconf
import gateway
import hub_api

##############################################################################
# Gateway Configuration and Main Loop Class
##############################################################################


class GatewayNode():
    '''
      Currently this just provides getup and go for the gateway.
    '''
    ##########################################################################
    # Init & Shutdown
    ##########################################################################

    def __init__(self):
        self._param = rocon_gateway.setup_ros_parameters()
        key = uuid.uuid4()  # random 16 byte string, alternatively uuid.getnode() returns a hash based on the mac address, uuid.uid1() based on localhost and time
        self._unique_name = self._param['name'] + key.hex  # append a unique hex string
        self._hub_manager = hub_api.HubManager(
                             hub_whitelist=self._param['hub_whitelist'],
                             hub_blacklist=self._param['hub_blacklist']
                             )
        self._gateway = gateway.Gateway(self._hub_manager, self._param, self._publish_gateway_info)
        self._gateway_services = self._setup_ros_services()
        self._gateway_publishers = self._setup_ros_publishers()
        self._hub_discovery_thread = zeroconf.HubDiscovery(self.hub_discovery_update)
        # Directly connect if a uri is configured
        if self._param['hub_uri'] != '':
            o = urlparse(self._param['hub_uri'])
            hub, unused_error_code = self._hub_manager.connect_to_hub_with_timeout(o.hostname, o.port)
            if hub:
                hub.register_gateway(self._param['firewall'],
                                     self._unique_name,
                                     self._gateway.remote_gateway_request_callbacks
                                     )

    def shutdown(self):
        '''
          Clears this gateway's information from the redis server.
        '''
        rospy.loginfo("Gateway : shutting down.")
        try:
            self._gateway.shutdown()
            self._hub_manager.shutdown()
            self._hub_discovery_thread.shutdown()
        except Exception as e:
            rospy.logerr("Gateway : error on shutdown [%s]" % str(e))

    ##########################################################################
    # Zeroconf
    ##########################################################################

    def hub_discovery_update(self, ip, port):
        rospy.loginfo("hub discovery update")
        hub, unused_error_code = self._hub_manager.connect_to_hub(ip, port)
        if hub:
            hub.register_gateway(self._param['firewall'],
                                 self._unique_name,
                                 self._gateway.remote_gateway_request_callbacks
                                 )

    ##########################################################################
    # Ros Pubs, Subs and Services
    ##########################################################################

    def _setup_ros_services(self):
        gateway_services = {}
        gateway_services['connect_hub']   = rospy.Service('~connect_hub',                   gateway_srvs.ConnectHub,       self.ros_service_connect_hub)
        gateway_services['remote_gateway_info']  = rospy.Service('~remote_gateway_info',    gateway_srvs.RemoteGatewayInfo,self.ros_service_remote_gateway_info)
        gateway_services['advertise']     = rospy.Service('~advertise',                     gateway_srvs.Advertise,        self._gateway.ros_service_advertise)
        gateway_services['advertise_all'] = rospy.Service('~advertise_all',                 gateway_srvs.AdvertiseAll,     self._gateway.ros_service_advertise_all)        
        gateway_services['flip']          = rospy.Service('~flip',                          gateway_srvs.Remote,    self._gateway.ros_service_flip)        
        gateway_services['flip_all']      = rospy.Service('~flip_all',                      gateway_srvs.RemoteAll, self._gateway.ros_service_flip_all)
        gateway_services['pull']          = rospy.Service('~pull',                          gateway_srvs.Remote,    self._gateway.ros_service_pull)        
        gateway_services['pull_all']      = rospy.Service('~pull_all',                      gateway_srvs.RemoteAll, self._gateway.ros_service_pull_all)
        return gateway_services

    def _setup_ros_publishers(self):
        gateway_publishers = {}
        gateway_publishers['gateway_info'] = rospy.Publisher('~gateway_info', gateway_msgs.GatewayInfo, latch=True)
        return gateway_publishers

    ##########################################################################
    # Ros Service Callbacks
    ##########################################################################

    def ros_service_connect_hub(self, request):
        '''
          Handle incoming requests to connect directly to a gateway hub.

          Requests are of the form of a uri (hostname:port pair) pointing to
          the gateway hub.
        '''
        response = gateway_srvs.ConnectHubResponse()
        o = urlparse(request.uri)
        response.result = self._connect(o.hostname, o.port)
        if response.result == gateway_msgs.ErrorCodes.SUCCESS:
            rospy.loginfo("Gateway : made direct connection to hub [%s]" % request.uri)
        elif response.result == gateway_msgs.ErrorCodes.HUB_CONNECTION_BLACKLISTED:
            response.error_message = "cowardly refusing your request since that hub is blacklisted [%s]." % request.uri
        elif response.result == gateway_msgs.ErrorCodes.HUB_CONNECTION_ALREADY_EXISTS:
            response.error_message = "(currently) can only connect to one hub at a time [%s]." % self.gateway.hub.name
        else:
            response.error_message = "direct connection failed, probably not available [%s]." % request.uri
        return response

    def _publish_gateway_info(self):
        gateway_info = gateway_msgs.GatewayInfo()
        gateway_info.name = self.gateway.unique_name
        if self.gateway._ip != None:
            gateway_info.ip = self.gateway._ip
        else:
            gateway_info.ip = 'unavailable'
        gateway_info.connected = self.gateway.is_connected
        gateway_info.hub_name = self.gateway.hub.name
        gateway_info.hub_uri = self.gateway.hub.uri
        gateway_info.firewall = self._param['firewall']
        gateway_info.flipped_connections = self.gateway.flipped_interface.get_flipped_connections()
        gateway_info.flipped_in_connections = self.gateway.flipped_interface.getLocalRegistrations()
        gateway_info.flip_watchlist = self.gateway.flipped_interface.getWatchlist()
        gateway_info.pulled_connections = self.gateway.pulled_interface.getLocalRegistrations()
        gateway_info.pull_watchlist = self.gateway.pulled_interface.getWatchlist()
        gateway_info.public_watchlist = self.gateway.public_interface.getWatchlist()
        gateway_info.public_interface = self.gateway.public_interface.getInterface()
        self._gateway_publishers['gateway_info'].publish(gateway_info)

    def ros_service_remote_gateway_info(self, request):
        response = gateway_srvs.RemoteGatewayInfoResponse()
        requested_gateways = request.gateways if request.gateways else self.gateway.hub.list_remote_gateway_names()
        for gateway in requested_gateways:
            remote_gateway_info = self.gateway.hub.remote_gateway_info(gateway)
            if remote_gateway_info:
                response.gateways.append(remote_gateway_info)
            else:
                rospy.logwarn("Gateway : requested gateway info for unavailable gateway [%s]" % gateway)
        return response

