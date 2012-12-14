#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway/LICENSE
#
##############################################################################
# Imports
##############################################################################

import roslib
roslib.load_manifest('rocon_gateway')
import rospy
import rocon_gateway
import redis
import gateway_msgs.msg
import gateway_msgs.srv
from urlparse import urlparse
import zeroconf_msgs.srv

##############################################################################
# Gateway Configuration and Main Loop Class
##############################################################################


class Gateway():
    '''
      Currently this just provides getup and go for the gateway.

        1. configure ros params
        2. setup ros pubsubs, services
        3. optionally setup zeroconf if necessary and available
        4. loop until a hub connection is made, then spin
        5. shutdown
    '''
    def __init__(self):
        self.gateway_sync = None  # hub and local ros master connection

        self.param = rocon_gateway.setup_ros_parameters()
        self.gateway_sync = rocon_gateway.GatewaySync(self.param)  # maybe pass in the whole params dictionary?
        self._gateway_services = self._setup_ros_services()

        self._zeroconf_services = {}
        if not self._attempt_direct_connection():
            self._zeroconf_services = rocon_gateway.zeroconf.setup_ros_services()

    ##########################################################################
    # Main Loop
    ##########################################################################

    def spin(self):
        previously_found_hubs = []
        while not rospy.is_shutdown() and not self.gateway_sync.is_connected:
            rospy.sleep(1.0)
            if self._zeroconf_services:
                self._scan_for_zeroconf_hubs(previously_found_hubs)
            else:
                # do nothing, just wait for a service call
                rospy.logdebug("Gateway : waiting for hub uri input.")

        rospy.loginfo("Gateway : connected to hub [%s][%s]." % (self.gateway_sync.unique_name, self.gateway_sync.hub.name))
        rospy.spin()
        self._shutdown()

    def _shutdown(self):
        '''
          Clears this gateway's information from the redis server.
        '''
        try:
            self.gateway_sync.shutdown()
        except Exception as e:
            rospy.logerr("Gateway : error on shutdown [%s]" % str(e))
        rospy.logdebug("Gateway : redis server cleared of gateway information")

    #############################################
    # Ros Pubs, Subs and Services
    #############################################

    def _setup_ros_services(self):
        gateway_services = {}
        gateway_services['connect_hub']   = rospy.Service('~connect_hub',                   gateway_msgs.srv.ConnectHub,       self.ros_service_connect_hub)
        gateway_services['gateway_info']  = rospy.Service('~gateway_info',                  gateway_msgs.srv.GatewayInfo,      self.ros_service_gateway_info)
        gateway_services['remote_gateway_info']  = rospy.Service('~remote_gateway_info',    gateway_msgs.srv.RemoteGatewayInfo,self.ros_service_remote_gateway_info)
        gateway_services['advertise']     = rospy.Service('~advertise',                     gateway_msgs.srv.Advertise,        self.gateway_sync.ros_service_advertise)
        gateway_services['advertise_all'] = rospy.Service('~advertise_all',                 gateway_msgs.srv.AdvertiseAll,     self.gateway_sync.ros_service_advertise_all)        
        gateway_services['flip']          = rospy.Service('~flip',                          gateway_msgs.srv.Remote,    self.gateway_sync.ros_service_flip)        
        gateway_services['flip_all']      = rospy.Service('~flip_all',                      gateway_msgs.srv.RemoteAll, self.gateway_sync.ros_service_flip_all)
        gateway_services['pull']          = rospy.Service('~pull',                          gateway_msgs.srv.Remote,    self.gateway_sync.ros_service_pull)        
        gateway_services['pull_all']      = rospy.Service('~pull_all',                      gateway_msgs.srv.RemoteAll, self.gateway_sync.ros_service_pull_all)
        return gateway_services

    #############################################
    # Ros Service Callbacks
    #############################################

    def ros_service_connect_hub(self, request):
        '''
          Incoming requests are used to then try and connect to the gateway hub
          if not already connected.

          Requests are of the form of a uri (hostname:port pair) pointing to
          the gateway hub.
        '''
        response = gateway_msgs.srv.ConnectHubResponse()
        o = urlparse(request.uri)
        response.result = self._connect(o.hostname, o.port)
        if response.result == gateway_msgs.msg.Result.SUCCESS:
            rospy.loginfo("Gateway : made direct connection to hub [%s]" % request.uri)
        elif response.result == gateway_msgs.msg.Result.HUB_CONNECTION_BLACKLISTED:
            response.error_message = "cowardly refusing your request since that hub is blacklisted [%s]." % request.uri
        elif response.result == gateway_msgs.msg.Result.HUB_CONNECTION_ALREADY_EXISTS:
            response.error_message = "(currently) can only connect to one hub at a time [%s]." % self.gateway_sync.hub.name
        else:
            response.error_message = "direct connection failed, probably not available [%s]." % request.uri
        return response

    def ros_service_gateway_info(self, msg):
        response = gateway_msgs.srv.GatewayInfoResponse()
        if self.gateway_sync.unique_name != None:
            response.name = self.gateway_sync.unique_name
        else:
            response.name = self.gateway_sync.unresolved_name
        if self.gateway_sync._ip != None:
            response.ip = self.gateway_sync._ip
        else:
            response.ip = 'unavailable'
        response.connected = self.gateway_sync.is_connected
        response.hub_name = self.gateway_sync.hub.name
        response.hub_uri = self.gateway_sync.hub.uri
        response.firewall = self.param['firewall']
        response.flipped_connections = self.gateway_sync.flipped_interface.get_flipped_connections()
        response.flipped_in_connections = self.gateway_sync.flipped_interface.getLocalRegistrations()
        response.flip_watchlist = self.gateway_sync.flipped_interface.getWatchlist()
        response.pulled_connections = self.gateway_sync.pulled_interface.getLocalRegistrations()
        response.pull_watchlist = self.gateway_sync.pulled_interface.getWatchlist()
        response.public_watchlist = self.gateway_sync.public_interface.getWatchlist()
        response.public_interface = self.gateway_sync.public_interface.getInterface()
        return response

    def ros_service_remote_gateway_info(self, request):
        response = gateway_msgs.srv.RemoteGatewayInfoResponse()
        requested_gateways = request.gateways if request.gateways else self.gateway_sync.hub.list_remote_gateway_names()
        for gateway in requested_gateways:
            remote_gateway_info = self.gateway_sync.hub.remote_gateway_info(gateway)
            if remote_gateway_info:
                response.gateways.append(remote_gateway_info)
            else:
                rospy.logwarn("Gateway : requested gateway info for unavailable gateway [%s]" % gateway)
        return response

    #############################################
    # Hub Connection Methods
    #############################################

    def _attempt_direct_connection(self):
        '''
          If configured with a static hub_uri, attempt a direct connection.

          @return success of the connection
          @rtype bool
        '''
        if self.param['hub_uri'] != '':
            o = urlparse(self.param['hub_uri'])
            if self._connect(o.hostname, o.port) == gateway_msgs.msg.Result.SUCCESS:
                rospy.loginfo("Gateway : made direct connection to hub [%s]" % self.param['hub_uri'])
                return True
            else:
                rospy.logwarn("Gateway : failed direct connection attempt to hub [%s]" % self.param['hub_uri'])
        return False

    def _scan_for_zeroconf_hubs(self, previously_found_hubs):
        '''
          Does a quick scan on zeroconf for gateway hubs. If new ones are
          found, and it is not on the blacklist, it attempts a connection.

          This gets run in the pre-spin part of the spin loop.

          @param previously_found_hubs: zeroconf names of previously scanned hubs
          @type  previously_found_hubs: list of str
        '''
        # Get discovered redis server list from zeroconf
        req = zeroconf_msgs.srv.ListDiscoveredServicesRequest()
        req.service_type = rocon_gateway.zeroconf.gateway_hub_service
        resp = self._zeroconf_services["list_discovered_services"](req)
        rospy.logdebug("Gateway : checking for autodiscovered gateway hubs")
        new_services = lambda l1,l2: [x for x in l1 if x not in l2]
        for service in new_services(resp.services,previously_found_hubs):
            previously_found_hubs.append(service)
            (ip, port) = rocon_gateway.zeroconf.resolve_address(service)
            rospy.loginfo("Gateway : discovered hub zeroconf service at " + str(ip) + ":"+str(service.port))
            connect_result = self._connect(ip,port)
            if connect_result == gateway_msgs.msg.Result.SUCCESS:
                break

    def _connect(self,ip,port):
        if self.gateway_sync.is_connected: 
            rospy.logwarn("Gateway : gateway is already connected, aborting connection attempt.")
            return gateway_msgs.msg.Result.HUB_CONNECTION_ALREADY_EXISTS
        try:
            hub_name = rocon_gateway.resolve_hub(ip,port)
            rospy.loginfo("Gateway : resolved hub name [%s].", hub_name)
        except redis.exceptions.ConnectionError:
            rospy.logerr("Gateway : couldn't connect to the hub [%s:%s]", ip, port)
            return gateway_msgs.msg.Result.HUB_CONNECTION_UNRESOLVABLE
        if ip in self.param['hub_blacklist']:
            rospy.loginfo("Gateway : ignoring blacklisted hub [%s]",ip)
            return gateway_msgs.msg.Result.HUB_CONNECTION_BLACKLISTED
        elif hub_name in self.param['hub_blacklist']:
            rospy.loginfo("Gateway : ignoring blacklisted hub [%s]",hub_name)
            return gateway_msgs.msg.Result.HUB_CONNECTION_BLACKLISTED
        # Handle whitelist (ip or hub name)
        if (len(self.param['hub_whitelist']) == 0) or (ip in self.param['hub_whitelist']) or (hub_name in self.param['hub_whitelist']):
            if self.gateway_sync.connect_to_hub(ip,port):
                return gateway_msgs.msg.Result.SUCCESS
            else:
                return gateway_msgs.msg.Result.HUB_CONNECTION_FAILED
        else:
            rospy.loginfo("Gateway : hub/ip not in non-empty whitelist [%s]",hub_name)
            return gateway_msgs.msg.Result.HUB_CONNECTION_NOT_IN_NONEMPTY_WHITELIST
