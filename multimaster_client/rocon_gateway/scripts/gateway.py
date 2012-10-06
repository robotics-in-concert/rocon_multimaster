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
import gateway_comms.srv
from std_msgs.msg import String
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
        self._hub_name = None # name of the gateway hub we are connected to
        self.gateway_sync = None # hub and local ros master connections
        
        self.param = rocon_gateway.setupRosParameters()
        self.gateway_sync = rocon_gateway.GatewaySync(self.param['name']) # redis server (hub) and local ros master connections
        self._gateway_services = self._setupRosServices()
        self._gateway_subscribers = self._setupRosSubscribers()

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
                rospy.logdebug("Gateway : waiting for hub uri input.")
                pass # add ip connect here

        # Once you get here, it should be connected to redis server
        rospy.loginfo("Gateway : connected to hub [%s][%s]."%(self.gateway_sync.unique_name,self._hub_name)) 
        rospy.spin()

        # When the node is going off, it should remove it's info from redis-server
        self._shutdown()

        
    def _shutdown(self):
        '''
          Clears this gateway's information from the redis server.
        '''
        try:
            self.gateway_sync.clearServer()
        except Exception as e:
            print str(e)
        rospy.loginfo("Gateway : server cleared");
    
    #############################################
    # Ros Pubs, Subs and Services
    #############################################

    def _setupRosSubscribers(self):
        gateway_subscribers = {}
        gateway_subscribers['connect'] = rospy.Subscriber("~connect",String,self.processConnectHubRequest)
        return gateway_subscribers

    def _setupRosServices(self):
        gateway_services = {}
        gateway_services['gateway_info']  = rospy.Service('~gateway_info',  gateway_comms.srv.GatewayInfo,  self.processGatewayInfo)
        gateway_services['advertise']     = rospy.Service('~advertise',     gateway_comms.srv.Advertise,    self.gateway_sync.advertise)
        gateway_services['advertise_all'] = rospy.Service('~advertise_all', gateway_comms.srv.AdvertiseAll, self.gateway_sync.advertiseAll)        
        gateway_services['flip']          = rospy.Service('~flip',          gateway_comms.srv.Flip,         self.gateway_sync.rosServiceFlip)        
        gateway_services['flip_pattern']  = rospy.Service('~flip_pattern',  gateway_comms.srv.FlipPatterns, self.gateway_sync.rosServiceFlipPattern)        
        gateway_services['flip_all']      = rospy.Service('~flip_all',      gateway_comms.srv.FlipAll,      self.gateway_sync.rosServiceFlipAll)
        return gateway_services        
     
    #############################################
    # Ros Service Callbacks
    #############################################

    def processConnectHubRequest(self,uri):
        '''
          Incoming requests are used to then try and connect to the gateway hub
          if not already connected.
          
          Requests are of the form of a uri (hostname:port pair) pointing to 
          the gateway hub. 
        '''
        if not self.gateway_sync.is_connected:
            if self._connectByUri(uri.data):
                rospy.logwarn("Gateway : made direct connection attempt to hub [%s]"%uri.data)
            else:
                rospy.logwarn("Gateway : failed direct connection attempt to hub [%s]"%uri.data)
        else:
            rospy.logwarn("Gateway : is already connected to a hub, cowardly refusing to connect.")

    def processGatewayInfo(self,msg):
        response = GatewayInfoResponse()
        # Should add something about connected status here
        if self.gateway_sync.unique_name != None:
            response.name = self.gateway_sync.unique_name
        else:
            response.name = self.gateway_sync.unresolved_name
        response.public_interface.topics = self.gateway_sync.master.public_interface['topic']
        response.public_interface.services = self.gateway_sync.master.public_interface['service']
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
        if self.gateway_sync.is_connected: 
            rospy.logwarn("Gateway : gateway is already connected, aborting connection attempt.")
            return False
        if self.param['hub_uri'] != '':
            if self._connectByUri(self.param['hub_uri']):
                rospy.logwarn("Gateway : made direct connection attempt to hub [%s]"%self.param['hub_uri'])
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
            rospy.loginfo("Gateway : discovered hub at " + str(ip) + ":"+str(service.port))
            try:
                self._hub_name = rocon_gateway.resolveHub(ip,port)
                rospy.loginfo("Gateway : resolved hub name [%s].", self._hub_name)
            except redis.exceptions.ConnectionError:
                rospy.logerr("Gateway : couldn't connect to the hub [%s:%s]", ip, port)
                continue
            # Check blacklist (ip or hub name)
            if ip in self.param['hub_blacklist']:
                rospy.loginfo("Gateway : ignoring blacklisted hub [%s]",ip)
                continue
            if self._hub_name in self.param['hub_blacklist']:
                rospy.loginfo("Gateway : ignoring blacklisted hub [%s]",self._hub_name)
                continue
            # Handle whitelist (ip or hub name)
            if len(self.param['hub_whitelist']) == 0:
                if self._connectByZeroconfName(service):
                    break
            elif ip in self.param['hub_whitelist']:
                if self._connectByZeroconfName(service):
                    break
            else:
                if self._hub_name in self.param['hub_whitelist']:
                    if self._connectByZeroconfName(service):
                        break

    def _connectByZeroconfName(self,msg):
        (ip, port) = rocon_gateway.zeroconf.resolveAddress(msg)
        return self._connect(ip,port)
        
    def _connectByUri(self,uri):
        o = urlparse(uri)
        return self._connect(o.hostname, o.port)
    
    def _connect(self,ip,port):
        if self.gateway_sync.connectToHub(ip,port):
            return True
        else:
            return False

##############################################################################
# Launch point
##############################################################################

if __name__ == '__main__':
    
    rospy.init_node('gateway')

    gateway = Gateway()
    rospy.loginfo("Gateway : initialised.")

    gateway.spin()
    rospy.loginfo("Gateway : shutting down.")

