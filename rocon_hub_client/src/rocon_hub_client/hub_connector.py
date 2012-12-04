import rospy
import rocon_gateway
import redis
from urlparse import urlparse

import zeroconf_msgs.srv
from gateway_msgs.msg import *

class HubConnector(object):
    hub_service = "_ros-multimaster-hub._tcp"
    is_connected = False

    def __init__(self,whitelist,blacklist,is_zeroconf,connectFunc):

        self.param = {} if self.param == None else self.param
            
        self.param['whitelist'] = whitelist
        self.param['blacklist'] = blacklist
        self.connectToHub = connectFunc

        self._zeroconf_services = {}
        if is_zeroconf:
            self._zeroconf_services = rocon_gateway.zeroconf.setupRosServices()

    def connect(self,hub_uri=None):
        if hub_uri:
            return self.directConnect(hub_uri)
        else:
            return self.zeroconfConnect()

    def directConnect(self,hub_uri):
        '''
            If hub_uri is provided, attempt a direct connection
        '''
        (ip, port) = hub_uri.split(':')

        if self._connect(ip, int(port)) == gateway_msgs.msg.Result.SUCCESS:
            rospy.loginfo("HubConnector : made directo connection to Hub [%s]"%hub_uri)
            self.is_connected = True
            return True
        else:
            rospy.logwarn("HubConnector : failed direct connection attempt to hub [%s]"%hub_uri)
            return False


    def zeroconfConnect(self):
        while not rospy.is_shutdown() and not self.is_connected:
            rospy.sleep(1.0)
            if self._zeroconf_services:
                found_hubs = self._scanForZeroconfHubs()
                for (ip, port) in found_hubs:
                    rospy.loginfo("HubConnector : discovered hub zeroconf service at " + str(ip) + ":" + str(port))
                    if self._connect(ip,port) == gateway_msgs.msg.Result.SUCCESS:
                        self.is_connected = True
                        break
                rospy.logdebug("HubConnector : waiting for hub uri input")

        return self.is_connected


    def _scanForZeroconfHubs(self):
        ''' 
            Does a quick scan on zeroconf for gateway hubs. If new ones are
            found, and it is not on the blacklist, it attempts a connection.
                                            
            This gets run in the pre-spin part of the spin loop.
                                                        
            @param previously_found_hubs: zeroconf names of previously scanned hubs
            @type  previously_found_hubs: list of str
        '''

        previously_found_hubs = []
        found_hubs = []
        # Get discovered redis server list from zeroconf
        req = zeroconf_msgs.srv.ListDiscoveredServicesRequest() 
        req.service_type = rocon_gateway.zeroconf.gateway_hub_service
        resp = self._zeroconf_services["list_discovered_services"](req)
        rospy.logdebug("HubConnector : checking for autodiscovered gateway hubs")
        new_services = lambda l1,l2: [x for x in l1 if x not in l2] 
                                        
        for service in new_services(resp.services,previously_found_hubs):
            (ip, port) = rocon_gateway.zeroconf.resolve_address(service)
            found_hubs.append((ip,port))

        return found_hubs

    def _connect(self,ip,port):
        if self.is_connected: 
            rospy.logwarn("HubConnector : gateway is already connected, aborting connection attempt.")
            return gateway_msgs.msg.Result.HUB_CONNECTION_ALREADY_EXISTS
        try:
            hub_name = rocon_gateway.resolve_hub(ip,port)
            rospy.loginfo("HubConnector : resolved hub name [%s].", hub_name)
        except redis.exceptions.ConnectionError:
            rospy.logerr("HubConnector : couldn't connect to the hub [%s:%s]", ip, port)
            return gateway_msgs.msg.Result.HUB_CONNECTION_UNRESOLVABLE
        if ip in self.param['blacklist']:
            rospy.loginfo("HubConnector : ignoring blacklisted hub [%s]",ip)
            return gateway_msgs.msg.Result.HUB_CONNECTION_BLACKLISTED
        elif hub_name in self.param['blacklist']:
            rospy.loginfo("HubConnector : ignoring blacklisted hub [%s]",hub_name)
            return gateway_msgs.msg.Result.HUB_CONNECTION_BLACKLISTED
        # Handle whitelist (ip or hub name)
        if (len(self.param['whitelist']) == 0) or (ip in self.param['whitelist']) or (hub_name in self.param['whitelist']):
            if self.connectToHub(ip,port):
                return gateway_msgs.msg.Result.SUCCESS
            else:
                return gateway_msgs.msg.Result.HUB_CONNECTION_FAILED
        else:
            rospy.loginfo("HubConnector : hub/ip not in non-empty whitelist [%s]",hub_name)
            return gateway_msgs.msg.Result.HUB_CONNECTION_NOT_IN_NONEMPTY_WHITELIST
