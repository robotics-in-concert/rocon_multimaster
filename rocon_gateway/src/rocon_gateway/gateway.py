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
import redis
import gateway_msgs.msg as gateway_msgs
import gateway_msgs.srv as gateway_srvs
from urlparse import urlparse

# Local imports
import zeroconf
import hub_sync
import hub_api

##############################################################################
# Gateway Configuration and Main Loop Class
##############################################################################


class Gateway():
    '''
      Currently this just provides getup and go for the gateway.
    '''
    ##########################################################################
    # Init & Shutdown
    ##########################################################################

    def __init__(self):
        self.param = rocon_gateway.setup_ros_parameters()
        self.gateway_sync = rocon_gateway.GatewaySync(self.param, self._publish_gateway_info)  # maybe pass in the whole params dictionary?
        self._gateway_services = self._setup_ros_services()
        self._gateway_publishers = self._setup_ros_publishers()
        self._hub_discovery_thread = zeroconf.HubDiscovery(self.hub_discovery_update)
        self._hub_sync = hub_sync.HubSync()

    def _shutdown(self):
        '''
          Clears this gateway's information from the redis server.
        '''
        rospy.loginfo("Gateway : shutting down.")
        try:
            #self.gateway_sync.shutdown()
            self._hub_sync.shutdown()
            self._hub_discovery_thread.shutdown()
        except Exception as e:
            rospy.logerr("Gateway : error on shutdown [%s]" % str(e))

    ##########################################################################
    # Zeroconf
    ##########################################################################

    def hub_discovery_update(self, ip, port):
        rospy.loginfo("hub discovery update")
        self._connect_to_hub(ip, port)

    #############################################
    # Hub Connection Methods
    #############################################

    def _attempt_direct_connection(self):
        '''
          If configured with a static hub_uri, attempt a direct connection.

          @return success of the connection
          @rtype bool
        '''
        connection_timeout = rospy.Duration(5.0)
        start_time = rospy.Time.now()
        if self.param['hub_uri'] != '':
            o = urlparse(self.param['hub_uri'])
            while not rospy.is_shutdown() and not (rospy.Time.now() - start_time > connection_timeout):
                if self._connect(o.hostname, o.port) == gateway_msgs.ErrorCodes.SUCCESS:
                    rospy.loginfo("Gateway : made direct connection to hub [%s]" % self.param['hub_uri'])
                    return True
                else:
                    rospy.sleep(0.3)
            rospy.logwarn("Gateway : couldn't connect to the hub (probably not up yet) [%s]" % self.param['hub_uri'])
        return False

    def _connect_to_hub(self, ip, port):
        try:
            hub_name = hub_api.resolve_hub(ip, port)
            rospy.loginfo("Gateway : resolved hub name [%s].", hub_name)
        except hub_api.HubUnavailableError:
            return gateway_msgs.ErrorCodes.HUB_CONNECTION_UNRESOLVABLE
        except hub_api.HubNameNotFoundError:
            return gateway_msgs.ErrorCodes.HUB_NAME_NOT_FOUND
        if ip in self.param['hub_blacklist']:
            rospy.loginfo("Gateway : ignoring blacklisted hub [%s]", ip)
            return gateway_msgs.ErrorCodes.HUB_CONNECTION_BLACKLISTED
        elif hub_name in self.param['hub_blacklist']:
            rospy.loginfo("Gateway : ignoring blacklisted hub [%s]", hub_name)
            return gateway_msgs.ErrorCodes.HUB_CONNECTION_BLACKLISTED
        # Handle whitelist (ip or hub name)
        if (len(self.param['hub_whitelist']) == 0) or (ip in self.param['hub_whitelist']) or (hub_name in self.param['hub_whitelist']):
            print("connect, the real deal")
            #if self.gateway_sync.connect_to_hub(ip, port):
            #    return gateway_msgs.ErrorCodes.SUCCESS
            #else:
            #    return gateway_msgs.ErrorCodes.HUB_CONNECTION_FAILED
        else:
            rospy.loginfo("Gateway : hub/ip not in non-empty whitelist [%s]", hub_name)
            return gateway_msgs.ErrorCodes.HUB_CONNECTION_NOT_IN_NONEMPTY_WHITELIST

#    def _connect(self, ip, port):
#        if self.gateway_sync.is_connected:
#            rospy.logwarn("Gateway : gateway is already connected, aborting connection attempt.")
#            return gateway_msgs.ErrorCodes.HUB_CONNECTION_ALREADY_EXISTS
#        try:
#            hub_name = rocon_gateway.resolve_hub(ip,port)
#            if hub_name:
#                rospy.loginfo("Gateway : resolved hub name [%s].", hub_name)
#            else:
#                return gateway_msgs.ErrorCodes.HUB_CONNECTION_UNRESOLVABLE  # could probably use a distinct error result for this instead.
#        except redis.exceptions.ConnectionError:
            #rospy.logwarn("Gateway : couldn't connect to the hub (probably not up yet) [%s:%s]", ip, port)
#            return gateway_msgs.ErrorCodes.HUB_CONNECTION_UNRESOLVABLE
#        if ip in self.param['hub_blacklist']:
#            rospy.loginfo("Gateway : ignoring blacklisted hub [%s]",ip)
#            return gateway_msgs.ErrorCodes.HUB_CONNECTION_BLACKLISTED
#        elif hub_name in self.param['hub_blacklist']:
#            rospy.loginfo("Gateway : ignoring blacklisted hub [%s]",hub_name)
#            return gateway_msgs.ErrorCodes.HUB_CONNECTION_BLACKLISTED
#        # Handle whitelist (ip or hub name)
#        if (len(self.param['hub_whitelist']) == 0) or (ip in self.param['hub_whitelist']) or (hub_name in self.param['hub_whitelist']):
#            if self.gateway_sync.connect_to_hub(ip,port):
#                return gateway_msgs.ErrorCodes.SUCCESS
#            else:
#                return gateway_msgs.ErrorCodes.HUB_CONNECTION_FAILED
#        else:
#            rospy.loginfo("Gateway : hub/ip not in non-empty whitelist [%s]",hub_name)
#            return gateway_msgs.ErrorCodes.HUB_CONNECTION_NOT_IN_NONEMPTY_WHITELIST

    ##########################################################################
    # Ros Pubs, Subs and Services
    ##########################################################################

    def _setup_ros_services(self):
        gateway_services = {}
        gateway_services['connect_hub']   = rospy.Service('~connect_hub',                   gateway_srvs.ConnectHub,       self.ros_service_connect_hub)
        gateway_services['remote_gateway_info']  = rospy.Service('~remote_gateway_info',    gateway_srvs.RemoteGatewayInfo,self.ros_service_remote_gateway_info)
        gateway_services['advertise']     = rospy.Service('~advertise',                     gateway_srvs.Advertise,        self.gateway_sync.ros_service_advertise)
        gateway_services['advertise_all'] = rospy.Service('~advertise_all',                 gateway_srvs.AdvertiseAll,     self.gateway_sync.ros_service_advertise_all)        
        gateway_services['flip']          = rospy.Service('~flip',                          gateway_srvs.Remote,    self.gateway_sync.ros_service_flip)        
        gateway_services['flip_all']      = rospy.Service('~flip_all',                      gateway_srvs.RemoteAll, self.gateway_sync.ros_service_flip_all)
        gateway_services['pull']          = rospy.Service('~pull',                          gateway_srvs.Remote,    self.gateway_sync.ros_service_pull)        
        gateway_services['pull_all']      = rospy.Service('~pull_all',                      gateway_srvs.RemoteAll, self.gateway_sync.ros_service_pull_all)
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
            response.error_message = "(currently) can only connect to one hub at a time [%s]." % self.gateway_sync.hub.name
        else:
            response.error_message = "direct connection failed, probably not available [%s]." % request.uri
        return response

    def _publish_gateway_info(self):
        gateway_info = gateway_msgs.GatewayInfo()
        gateway_info.name = self.gateway_sync.unique_name
        if self.gateway_sync._ip != None:
            gateway_info.ip = self.gateway_sync._ip
        else:
            gateway_info.ip = 'unavailable'
        gateway_info.connected = self.gateway_sync.is_connected
        gateway_info.hub_name = self.gateway_sync.hub.name
        gateway_info.hub_uri = self.gateway_sync.hub.uri
        gateway_info.firewall = self.param['firewall']
        gateway_info.flipped_connections = self.gateway_sync.flipped_interface.get_flipped_connections()
        gateway_info.flipped_in_connections = self.gateway_sync.flipped_interface.getLocalRegistrations()
        gateway_info.flip_watchlist = self.gateway_sync.flipped_interface.getWatchlist()
        gateway_info.pulled_connections = self.gateway_sync.pulled_interface.getLocalRegistrations()
        gateway_info.pull_watchlist = self.gateway_sync.pulled_interface.getWatchlist()
        gateway_info.public_watchlist = self.gateway_sync.public_interface.getWatchlist()
        gateway_info.public_interface = self.gateway_sync.public_interface.getInterface()
        self._gateway_publishers['gateway_info'].publish(gateway_info)

    def ros_service_remote_gateway_info(self, request):
        response = gateway_srvs.RemoteGatewayInfoResponse()
        requested_gateways = request.gateways if request.gateways else self.gateway_sync.hub.list_remote_gateway_names()
        for gateway in requested_gateways:
            remote_gateway_info = self.gateway_sync.hub.remote_gateway_info(gateway)
            if remote_gateway_info:
                response.gateways.append(remote_gateway_info)
            else:
                rospy.logwarn("Gateway : requested gateway info for unavailable gateway [%s]" % gateway)
        return response



#    def spin(self):
#        previously_found_hubs = []
#        while not rospy.is_shutdown() and not self.gateway_sync.is_connected:
#            rospy.sleep(1.0)
#            if self._zeroconf_services:
#                self._scan_for_zeroconf_hubs(previously_found_hubs)
#            else:
#                # do nothing, just wait for a service call
#                rospy.logdebug("Gateway : waiting for hub uri input.")
#        rospy.spin()
#        self._shutdown()
