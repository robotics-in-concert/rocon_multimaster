#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway/LICENSE 
#

##############################################################################
# Imports
##############################################################################

import roslib; roslib.load_manifest('rocon_gateway')
import rospy
import rocon_gateway
import redis
import gateway_comms.msg
import gateway_comms.srv
from urlparse import urlparse
import zeroconf_comms.srv

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
        self.gateway_sync = None # hub and local ros master connection
        
        self.param = rocon_gateway.setupRosParameters()
        self.gateway_sync = rocon_gateway.GatewaySync(self.param) # maybe pass in the whole params dictionary?
        self._gateway_services = self._setupRosServices()

        self._zeroconf_services = {}
        if not self._attemptDirectConnection():
            self._zeroconf_services = rocon_gateway.zeroconf.setupRosServices()

    ##########################################################################
    # Main Loop
    ##########################################################################

    def spin(self):
        previously_found_hubs = []
        while not rospy.is_shutdown() and not self.gateway_sync.is_connected:
            rospy.sleep(1.0)
            if self._zeroconf_services:
                self._scanForZeroconfHubs(previously_found_hubs)
            else:
                # do nothing, just wait for a service call
                rospy.logdebug("Gateway : waiting for hub uri input.")

        rospy.loginfo("Gateway : connected to hub [%s][%s]."%(self.gateway_sync.unique_name,self.gateway_sync.hub.name)) 
        rospy.spin()
        self._shutdown()

        
    def _shutdown(self):
        '''
          Clears this gateway's information from the redis server.
        '''
        try:
            self.gateway_sync.shutdown()
        except Exception as e:
            print str(e)
        rospy.loginfo("Gateway : server cleared");
    
    #############################################
    # Ros Pubs, Subs and Services
    #############################################

    def _setupRosServices(self):
        gateway_services = {}
        gateway_services['connect_hub']   = rospy.Service('~connect_hub',                   gateway_comms.srv.ConnectHub,       self.rosServiceConnectHub)
        gateway_services['gateway_info']  = rospy.Service('~gateway_info',                  gateway_comms.srv.GatewayInfo,      self.rosServiceGatewayInfo)
        gateway_services['remote_gateway_info']  = rospy.Service('~remote_gateway_info',    gateway_comms.srv.RemoteGatewayInfo,self.rosServiceRemoteGatewayInfo)
        gateway_services['advertise']     = rospy.Service('~advertise',                     gateway_comms.srv.Advertise,        self.gateway_sync.rosServiceAdvertise)
        gateway_services['advertise_all'] = rospy.Service('~advertise_all',                 gateway_comms.srv.AdvertiseAll,     self.gateway_sync.rosServiceAdvertiseAll)        
        gateway_services['flip']          = rospy.Service('~flip',                          gateway_comms.srv.RemoteRequest,    self.gateway_sync.rosServiceFlip)        
        gateway_services['flip_all']      = rospy.Service('~flip_all',                      gateway_comms.srv.RemoteRequestAll, self.gateway_sync.rosServiceFlipAll)
        gateway_services['pull']          = rospy.Service('~pull',                          gateway_comms.srv.RemoteRequest,    self.gateway_sync.rosServicePull)        
        gateway_services['pull_all']      = rospy.Service('~pull_all',                      gateway_comms.srv.RemoteRequestAll, self.gateway_sync.rosServicePullAll)
        return gateway_services        
     
    #############################################
    # Ros Service Callbacks
    #############################################

    def rosServiceConnectHub(self,request):
        '''
          Incoming requests are used to then try and connect to the gateway hub
          if not already connected.
          
          Requests are of the form of a uri (hostname:port pair) pointing to 
          the gateway hub. 
        '''
        response = gateway_comms.srv.ConnectHubResponse()
        o = urlparse(request.uri)
        response.result = self._connect(o.hostname, o.port)
        if response.result == gateway_comms.msg.Result.SUCCESS:
            rospy.loginfo("Gateway : made direct connection to hub [%s]"%request.uri)
        elif response.result == gateway_comms.msg.Result.HUB_CONNECTION_BLACKLISTED:
            response.error_message = "cowardly refusing your request since that hub is blacklisted [%s]."%request.uri
        elif response.result == gateway_comms.msg.Result.HUB_CONNECTION_ALREADY_EXISTS:
            response.error_message = "(currently) can only connect to one hub at a time [%s]."%self.gateway_sync.hub.name
        else:
            response.error_message = "direct connection failed, probably not available [%s]."%request.uri
        return response

    def rosServiceGatewayInfo(self,msg):
        response = gateway_comms.srv.GatewayInfoResponse()
        # Should add something about connected status here
        if self.gateway_sync.unique_name != None:
            response.name = self.gateway_sync.unique_name
        else:
            response.name = self.gateway_sync.unresolved_name
        response.connected = self.gateway_sync.is_connected
        response.hub_name = self.gateway_sync.hub.name
        for connection_type in rocon_gateway.connection_types:
            response.flipped_connections.extend(self.gateway_sync.flipped_interface.flipped[connection_type])
            response.flipped_in_connections.extend(self.gateway_sync.flipped_interface.flippedInConnections(connection_type))
            response.flip_watchlist.extend(self.gateway_sync.flipped_interface.watchlist[connection_type])
        # response message must have string output
        for flip in response.flip_watchlist:
            if not flip.rule.node:
                flip.rule.node = 'None'
        response.public_watchlist = self.gateway_sync.public_interface.getWatchlist()
        response.public_interface = self.gateway_sync.public_interface.getInterface()
        return response
    
    def rosServiceRemoteGatewayInfo(self,request):
        
        # Figure out names of relevant gateways
        requested_gateways = request.gateways
        available_gateways = self.gateway_sync.hub.listGateways()
        available_gateways = [x for x in available_gateways if x != self.gateway_sync.unique_name]
        if len(requested_gateways) == 0:
            gateways = available_gateways
        else:
            gateways = [x for x in requested_gateways if x in available_gateways]

        response = gateway_comms.srv.RemoteGatewayInfoResponse()
        # Public Interface
        public_interfaces = self.gateway_sync.hub.listPublicInterfaces(gateways)
        for key in public_interfaces:
            gateway_response = gateway_comms.msg.RemoteGateway(key, public_interfaces[key], [], [])
            response.gateways.append(gateway_response)
        return response

    #############################################
    # Hub Connection Methods
    #############################################

    def _attemptDirectConnection(self):
        '''
          If configured with a static hub_uri, attempt a direct connection.
          
          @return success of the connection
          @rtype bool
        '''
        if self.param['hub_uri'] != '':
            o = urlparse(self.param['hub_uri'])
            if self._connect(o.hostname, o.port) == gateway_comms.msg.Result.SUCCESS:
                rospy.loginfo("Gateway : made direct connection to hub [%s]"%self.param['hub_uri'])
                return True
            else:
                rospy.logwarn("Gateway : failed direct connection attempt to hub [%s]"%self.param['hub_uri'])
        return False

    def _scanForZeroconfHubs(self, previously_found_hubs):
        '''
          Does a quick scan on zeroconf for gateway hubs. If new ones are
          found, and it is not on the blacklist, it attempts a connection.
          
          This gets run in the pre-spin part of the spin loop.
          
          @param previously_found_hubs: zeroconf names of previously scanned hubs
          @type  previously_found_hubs: list of str
        '''
        # Get discovered redis server list from zeroconf
        req = zeroconf_comms.srv.ListDiscoveredServicesRequest() 
        req.service_type = rocon_gateway.zeroconf.gateway_hub_service
        resp = self._zeroconf_services["list_discovered_services"](req)
        rospy.logdebug("Gateway : checking for autodiscovered gateway hubs")
        new_services = lambda l1,l2: [x for x in l1 if x not in l2]
        for service in new_services(resp.services,previously_found_hubs):
            previously_found_hubs.append(service)
            (ip, port) = rocon_gateway.zeroconf.resolveAddress(service)
            rospy.loginfo("Gateway : discovered hub zeroconf service at " + str(ip) + ":"+str(service.port))
            connect_result = self._connect(ip,port)
            if connect_result == gateway_comms.msg.Result.SUCCESS:
                break

    def _connect(self,ip,port):
        if self.gateway_sync.is_connected: 
            rospy.logwarn("Gateway : gateway is already connected, aborting connection attempt.")
            return gateway_comms.msg.Result.HUB_CONNECTION_ALREADY_EXISTS
        try:
            hub_name = rocon_gateway.resolveHub(ip,port)
            rospy.logdebug("Gateway : resolved hub name [%s].", hub_name)
        except redis.exceptions.ConnectionError:
            rospy.logerr("Gateway : couldn't connect to the hub [%s:%s]", ip, port)
            return gateway_comms.msg.Result.HUB_CONNECTION_UNRESOLVABLE
        if ip in self.param['hub_blacklist']:
            rospy.loginfo("Gateway : ignoring blacklisted hub [%s]",ip)
            return gateway_comms.msg.Result.HUB_CONNECTION_BLACKLISTED
        elif hub_name in self.param['hub_blacklist']:
            rospy.loginfo("Gateway : ignoring blacklisted hub [%s]",hub_name)
            return gateway_comms.msg.Result.HUB_CONNECTION_BLACKLISTED
        # Handle whitelist (ip or hub name)
        if (len(self.param['hub_whitelist']) == 0) or (ip in self.param['hub_whitelist']) or (hub_name in self.param['hub_whitelist']):
            if self.gateway_sync.connectToHub(ip,port):
                return gateway_comms.msg.Result.SUCCESS
            else:
                return gateway_comms.msg.Result.HUB_CONNECTION_FAILED
        else:
            rospy.loginfo("Gateway : hub/ip not in non-empty whitelist [%s]",hub_name)
            return gateway_comms.msg.Result.HUB_CONNECTION_NOT_IN_NONEMPTY_WHITELIST

##############################################################################
# Launch point
##############################################################################

if __name__ == '__main__':
    
    rospy.init_node('gateway')
    gateway = Gateway()
    gateway.spin()
    rospy.loginfo("Gateway : shutting down.")

