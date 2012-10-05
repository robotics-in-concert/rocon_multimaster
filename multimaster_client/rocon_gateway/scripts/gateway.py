#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway/LICENSE 
#

import roslib; roslib.load_manifest('rocon_gateway')
import rospy
import rosgraph
import rocon_gateway
from gateway_comms.msg import *
from gateway_comms.srv import *
from zeroconf_comms.srv import *
from rocon_gateway import *
from rocon_gateway.utils import parseConnectionsFromFile
from std_msgs.msg import String
from urlparse import urlparse

class Gateway():
    '''
      This class is wrapper ros class of gateway sync.
      The role of this node is below:
      1. configure connections to gateway hubs
      2. make use of zeroconf for connections if flagged (optional)
      3. listen for requests (connections, flips, advertising) from the local ros system
      4. listen for requests (flips) from remote gateways
    '''
    def __init__(self):
        # Gateway configuration
        self.gateway_sync = None # hub and local ros master connections
        self.param = {}
        self.callbacks = {}
        self._hub_name = None # name of the gateway hub we are connecting to
        
        # Gateway Ros Api (topics/services/actions)
        self.gateway_service_names = {}
        self.gateway_services = {}
        gateway_request_service_name = "~request"
        gateway_connect_subscriber_name = "~connect"
        
        self.gateway_services['gateway_info'] = rospy.Service('~gateway_info',GatewayInfo,self.processGatewayInfo)
        self.gateway_services['list_public_interfaces'] = rospy.Service('~list_public_interfaces',ListPublicInterfaces,self.processListPublicInterfaces)
        
        # Optional Zeroconf Configuration
        self._zeroconf = False
        self._zeroconf_service = "_ros-gateway-hub._tcp"
        zeroconf_timeout = 5 # Amount of time to wait for the zeroconf services to appear
        zeroconf_add_listener_service = "zeroconf/add_listener"
        zeroconf_connection_service = "zeroconf/list_discovered_services"

        self.param = rocon_gateway.rosParameters()
        self.gateway_sync = GatewaySync(self.param['name']) # redis server (hub) and local ros master connections
        # Setup default public interface watchlist
        if self.param['default_public_interface'] != '':
            rospy.loginfo('Gateway : Parsing default public interface from file [%s]'%self.param['default_public_interface']);
            default_public_interface = parseConnectionsFromFile(self.param['default_public_interface'])
            self.gateway_sync.setPublicWatchlist(default_public_interface)
        else:
            rospy.logwarn('Gateway : No default public interface provided!')

        # Setup default blacklist to use (when explicit blacklist not supplied
        if self.param['blacklist'] != '':
            rospy.loginfo('Gateway : Parsing connection blacklist from file [%s]'%self.param['blacklist']);
            blacklist = parseConnectionsFromFile(self.param['blacklist'])
            self.gateway_sync.setDefaultBlacklist(blacklist)
        else:
            rospy.logwarn('Gateway : No default blacklist provided!')

        self.setupCallbacks()

        # Service Server for local node requests
        self.remote_list_service = rospy.Service(gateway_request_service_name,PublicHandler,self.processLocalRequest)

        self.connect_hub_subscriber = rospy.Subscriber(gateway_connect_subscriber_name,String,self.processConnectHubRequest)

        if self.param['hub_uri'] != '':
            if self.connectByUri(self.param['hub_uri']):
                rospy.logwarn("Gateway : made direct connection attempt to hub [%s]"%self.param['hub_uri'])
            else:
                rospy.logwarn("Gateway : failed direct connection attempt to hub [%s]"%self.param['hub_uri'])
        else: # see if we can use zeroconf to autodect
            rospy.loginfo("Gateway : waiting for zeroconf service to come up...")
            try:
                rospy.wait_for_service(zeroconf_add_listener_service, timeout=zeroconf_timeout)
                self._zeroconf = True
            except rospy.ROSException:
                rospy.logwarn("Gateway : timed out waiting for zeroconf services to come up.")

            if self._zeroconf:
                zeroconf_add_listener = rospy.ServiceProxy(zeroconf_add_listener_service,AddListener)
                self._zeroconf_service_proxy = rospy.ServiceProxy(zeroconf_connection_service,ListDiscoveredServices)
                if not zeroconf_add_listener(service_type = self._zeroconf_service):
                    self._zeroconf = False

    def setupCallbacks(self):

        # Individual callbacks, directly hooked into the gateway sync
        self.gateway_services['advertise'] = rospy.Service('~advertise',Advertise,self.gateway_sync.advertise)
        self.gateway_services['advertise_all'] = rospy.Service('~advertise_all',AdvertiseAll,self.gateway_sync.advertiseAll)        
        self.gateway_services['flip'] = rospy.Service('~flip',Flip,self.gateway_sync.rosServiceFlip)        
        self.gateway_services['flip_pattern'] = rospy.Service('~flip_pattern',FlipPattern,self.gateway_sync.rosServiceFlipPattern)        
        self.gateway_services['flip_all'] = rospy.Service('~flip_all',FlipAll,self.gateway_sync.rosServiceFlipAll)        
        
        self.callbacks["add_public_topic"] = self.gateway_sync.advertiseOld
        self.callbacks["add_public_service"] = self.gateway_sync.advertiseOld
        self.callbacks["remove_public_topic"] = self.gateway_sync.unadvertiseOld
        self.callbacks["remove_public_service"] = self.gateway_sync.unadvertiseOld

        self.callbacks["register_foreign_topic"] = self.gateway_sync.pull
        self.callbacks["unregister_foreign_topic"] = self.gateway_sync.unpull

        self.callbacks["register_foreign_service"] = self.gateway_sync.pull
        self.callbacks["unregister_foreign_service"] = self.gateway_sync.unpull

        self.callbacks["make_all_public"] = self.gateway_sync.makeAllPublic
        self.callbacks["remove_all_public"] = self.gateway_sync.removeAllPublic
     
    ##########################################################################
    # Ros Service Callbacks
    ##########################################################################
    
    def processLocalRequest(self,request):
        command = request.command
        success = False
        resp = PublicHandlerResponse()
        resp.success = success

        if command not in self.callbacks.keys():
            print "Wrong Command = " + str(command)
            return resp

        try:
            success, lists = self.callbacks[command](request.list)
        except Exception as e:
            print str(e)
            return resp

        resp.success = success

        return resp

    def processConnectHubRequest(self,uri):
        '''
          Incoming requests are used to then try and connect to the gateway hub
          if not already connected.
          
          Requests are of the form of a uri (hostname:port pair) pointing to 
          the gateway hub. 
        '''
        if not self.gateway_sync.is_connected:
            if self.connectByUri(uri.data):
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
        

    def processListPublicInterfaces(self,request):
        '''
          Returns a list of all public interfaces found advertised on the hub.
        '''
        response = ListPublicInterfacesResponse()
        public_interfaces = self.gateway_sync.hub.listPublicInterfaces()

        for gateway_name in public_interfaces.keys():
            public_interface = PublicInterface()
            public_interface.gateway_name = gateway_name
            interface = Interface()
            interface.topics = public_interfaces[gateway_name]['topic']
            interface.services = public_interfaces[gateway_name]['service']
            public_interface.interface = interface
            response.public_interfaces.append(public_interface)
        return response

    # It clears this client's information from redis-server
    def clearServer(self):
        try:
            self.gateway_sync.clearServer()
        except Exception as e:
            print str(e)
        rospy.loginfo("Gateway : server cleared");
    
    ##########################################################################
    # Connection Handlers
    ##########################################################################

    def scanForZeroconfHubs(self, previously_found_hubs):
        '''
          Does a quick scan on zeroconf for gateway hubs. If new ones are
          found, and it is not on the blacklist, it attempts a connection.
          
          This gets run in the pre-spin part of the spin loop.
          
          @param previously_found_hubs: zeroconf names of previously scanned hubs
          @type  previously_found_hubs: list of str
        '''
        # Get discovered redis server list from zeroconf
        req = ListDiscoveredServicesRequest() 
        req.service_type = self._zeroconf_service
        resp = self._zeroconf_service_proxy(req)
        rospy.logdebug("Gateway : checking for autodiscovered gateway hubs")
        new_services = lambda l1,l2: [x for x in l1 if x not in l2]
        for service in new_services(resp.services,previously_found_hubs):
            previously_found_hubs.append(service)
            (ip, port) = rocon_gateway.resolveZeroconfAddress(service)
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
                if self.connectByZeroconfName(service):
                    break
            elif ip in self.param['hub_whitelist']:
                if self.connectByZeroconfName(service):
                    break
            else:
                if self._hub_name in self.param['hub_whitelist']:
                    if self.connectByZeroconfName(service):
                        break

    def connectByZeroconfName(self,msg):
        (ip, port) = rocon_gateway.resolveZeroconfAddress(msg)
        return self.connect(ip,port)
        
    def connectByUri(self,uri):
        o = urlparse(uri)
        return self.connect(o.hostname, o.port)
    
    def connect(self,ip,port):
        if self.gateway_sync.connectToHub(ip,port):
            return True
        else:
            return False

    ##########################################################################
    # Connection Handlers
    ##########################################################################

    def spin(self):
        previously_found_hubs = []
        while not rospy.is_shutdown() and not self.gateway_sync.is_connected:
            if self._zeroconf:
                self.scanForZeroconfHubs(previously_found_hubs)
            else:
                rospy.logdebug("Gateway : waiting for hub uri input.")
                pass # add ip connect here
            rospy.sleep(3.0)

        # Once you get here, it is connected to redis server
        rospy.loginfo("Gateway : connected to hub [%s][%s]."%(self.gateway_sync.unique_name,self._hub_name)) 
        rospy.spin()

        # When the node is going off, it should remove it's info from redis-server
        self.clearServer()

if __name__ == '__main__':
    
    rospy.init_node('gateway')

    gateway = Gateway()
    rospy.loginfo("Gateway : initialised.")

    gateway.spin()
    rospy.loginfo("Gateway : shutting down.")

