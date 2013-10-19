#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/hydro-devel/rocon_gateway/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import rocon_gateway
import uuid
import gateway_msgs.msg as gateway_msgs
import gateway_msgs.srv as gateway_srvs
import std_msgs.msg as std_msgs
from urlparse import urlparse
import rocon_hub_client

# Local imports
import gateway
import hub_manager

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
        if self._param['disable_uuids']:
            self._unique_name = self._param['name']
            rospy.logwarn("Gateway : uuid's disabled, using possibly non-unique name [%s]" % self._unique_name)
        else:  # append a unique hex string
            key = uuid.uuid4()  # random 16 byte string, alternatively uuid.getnode() returns a hash based on the mac address, uuid.uid1() based on localhost and time
            self._unique_name = self._param['name'] + key.hex
            rospy.loginfo("Gateway : generated unique hash name [%s]" % self._unique_name)
        self._hub_manager = hub_manager.HubManager(
                             hub_whitelist=self._param['hub_whitelist'],
                             hub_blacklist=self._param['hub_blacklist']
                             )
        # Be careful of the construction sequence here, parts depend on others.
        self._gateway_publishers = self._setup_ros_publishers()
        self._gateway = gateway.Gateway(self._hub_manager, self._param, self._unique_name, self._publish_gateway_info)  # self._publish_gateway_info needs self._gateway_publishers
        self._gateway_services = self._setup_ros_services()  # Needs self._gateway
        self._gateway_subscribers = self._setup_ros_subscribers()  # Needs self._gateway
        direct_hub_uri_list = [self._param['hub_uri']] if self._param['hub_uri'] != '' else []
        self._hub_discovery_thread = rocon_hub_client.HubDiscovery(self._register_gateway, direct_hub_uri_list, self._param['disable_zeroconf'])

        # aliases
        self.spin = self._gateway.spin

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
            rospy.logerr("Gateway : unknown error on shutdown [%s][%s]" % (str(e), type(e)))
            raise

    ##########################################################################
    # Hub Discovery & Connection
    ##########################################################################

    def _register_gateway(self, ip, port):
        '''
          Called when either the hub discovery module finds a hub
          or a request to connect via ros service is made.

          It starts the actual redis connection with the hub and also
          registers the appropriate information about the gateway on
          the hub.

          Note, the return type is only really used by the service callback
          (ros_service_connect_hub).

          @return error code and message
          @rtype gateway_msgs.ErrorCodes, string

          @sa hub_discovery.HubDiscovery
        '''
        hub, error_code, error_code_str = self._hub_manager.connect_to_hub(ip, port)
        if hub:
            hub.register_gateway(self._param['firewall'],
                                 self._unique_name,
                                 self._gateway.remote_gateway_request_callbacks,
                                 self._gateway.disengage_hub,  # hub connection lost hook
                                 self._gateway.ip
                                 )
            rospy.loginfo("Gateway : registering on the hub [%s]" % hub.name)
            self._publish_gateway_info()
        else:
            rospy.logwarn("Gateway : not registering on the hub [%s]" % error_code_str)
        return error_code, error_code_str

    ##########################################################################
    # Ros Pubs, Subs and Services
    ##########################################################################

    def _setup_ros_services(self):
        gateway_services = {}
        gateway_services['connect_hub']         = rospy.Service('~connect_hub',         gateway_srvs.ConnectHub,       self.ros_service_connect_hub) #@IgnorePep8
        gateway_services['remote_gateway_info'] = rospy.Service('~remote_gateway_info', gateway_srvs.RemoteGatewayInfo,self.ros_service_remote_gateway_info) #@IgnorePep8
        gateway_services['advertise']           = rospy.Service('~advertise',           gateway_srvs.Advertise,        self._gateway.ros_service_advertise) #@IgnorePep8
        gateway_services['advertise_all']       = rospy.Service('~advertise_all',       gateway_srvs.AdvertiseAll,     self._gateway.ros_service_advertise_all) #@IgnorePep8
        gateway_services['flip']                = rospy.Service('~flip',                gateway_srvs.Remote,           self._gateway.ros_service_flip) #@IgnorePep8
        gateway_services['flip_all']            = rospy.Service('~flip_all',            gateway_srvs.RemoteAll,        self._gateway.ros_service_flip_all) #@IgnorePep8
        gateway_services['pull']                = rospy.Service('~pull',                gateway_srvs.Remote,           self._gateway.ros_service_pull) #@IgnorePep8
        gateway_services['pull_all']            = rospy.Service('~pull_all',            gateway_srvs.RemoteAll,        self._gateway.ros_service_pull_all) #@IgnorePep8
        gateway_services['set_watcher_period']  = rospy.Service('~set_watcher_period',  gateway_srvs.SetWatcherPeriod, self._gateway.ros_service_set_watcher_period) #@IgnorePep8
        return gateway_services

    def _setup_ros_publishers(self):
        gateway_publishers = {}
        gateway_publishers['gateway_info'] = rospy.Publisher('~gateway_info', gateway_msgs.GatewayInfo, latch=True)
        return gateway_publishers

    def _setup_ros_subscribers(self):
        gateway_subscribers = {}
        gateway_subscribers['force_update'] = rospy.Subscriber('~force_update', std_msgs.Empty, self._gateway.ros_subscriber_force_update)
        return gateway_subscribers

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
        response.result, response.error_message = self._register_gateway(o.hostname, o.port)
        # Some ros logging
        if response.result == gateway_msgs.ErrorCodes.SUCCESS:
            rospy.loginfo("Gateway : made direct connection to hub [%s]" % request.uri)
        return response

    def _publish_gateway_info(self):
        gateway_info = gateway_msgs.GatewayInfo()
        gateway_info.name = self._unique_name
        gateway_info.ip = self._gateway.ip
        gateway_info.connected = self._gateway.is_connected()
        gateway_info.hub_names = []
        gateway_info.hub_uris = []
        for hub in self._hub_manager.hubs:
            gateway_info.hub_names.append(hub.name)
            gateway_info.hub_uris.append(hub.uri)
        gateway_info.firewall = self._param['firewall']
        gateway_info.flipped_connections = self._gateway.flipped_interface.get_flipped_connections()
        gateway_info.flipped_in_connections = self._gateway.flipped_interface.getLocalRegistrations()
        gateway_info.flip_watchlist = self._gateway.flipped_interface.getWatchlist()
        gateway_info.pulled_connections = self._gateway.pulled_interface.getLocalRegistrations()
        gateway_info.pull_watchlist = self._gateway.pulled_interface.getWatchlist()
        gateway_info.public_watchlist = self._gateway.public_interface.getWatchlist()
        gateway_info.public_interface = self._gateway.public_interface.getInterface()
        self._gateway_publishers['gateway_info'].publish(gateway_info)

    def ros_service_remote_gateway_info(self, request):
        response = gateway_srvs.RemoteGatewayInfoResponse()
        requested_gateways = request.gateways if request.gateways else self._hub_manager.list_remote_gateway_names()
        for gateway in list(set(requested_gateways)):
            remote_gateway_info = self._hub_manager.remote_gateway_info(gateway)
            if remote_gateway_info:
                response.gateways.append(remote_gateway_info)
            else:
                rospy.logwarn("Gateway : requested gateway info for unavailable gateway [%s]" % gateway)
        return response
