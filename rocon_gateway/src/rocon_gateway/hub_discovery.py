#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/hydro-devel/rocon_gateway/LICENSE
#

###############################################################################
# Imports
###############################################################################

import threading
from urlparse import urlparse
import rospy
import zeroconf_msgs.srv as zeroconf_srvs

# local imports
import hub_api

###############################################################################
# Thread
###############################################################################


class HubDiscovery(threading.Thread):

    gateway_hub_service = "_ros-multimaster-hub._tcp"

    '''
      Used to discover hubs via zeroconf.
    '''
    def __init__(self, external_discovery_update_hook, direct_hub_uri_list=[]):
        '''
          @param external_discovery_update is a callback function that takes action on a discovery
          @type gateway_node.update_discovery_hook(ip, port)

          @param direct_hub_uri_list : list of uri's to hubs (e.g. http://localhost:6380
          @type list of uri
        '''
        threading.Thread.__init__(self)
        self.discovery_update_hook = external_discovery_update_hook
        self._trigger_shutdown = False
        self._direct_hub_uri_list = direct_hub_uri_list
        self._zeroconf_services_available = _zeroconf_services_available()
        if self._zeroconf_services_available:
            self._discovery_request = zeroconf_srvs.ListDiscoveredServicesRequest()
            self._discovery_request.service_type = HubDiscovery.gateway_hub_service
            _add_listener()
            self._list_discovered_services = rospy.ServiceProxy("zeroconf/list_discovered_services", zeroconf_srvs.ListDiscoveredServices)
            self._zeroconf_discovered_hubs = []
        # Only run the thread if we need to.
        if self._zeroconf_services_available or self._direct_hub_uri_list:
            self.start()

    def shutdown(self):
        '''
          Called from the main program to shutdown this thread.
        '''
        self._trigger_shutdown = True
        if self.is_alive():  # python complains if you join a non-started thread
            self.join()  # wait for the thread to finish

    def run(self):
        '''
          The hub discovery thread worker function. Monitors zeroconf for the presence of new hubs.
        '''
        self._internal_sleep_period = rospy.Duration(0, 200000000)  # 200ms
        while not rospy.is_shutdown() and not self._trigger_shutdown:
            # Zeroconf scanning
            if self._zeroconf_services_available:
                new_services, unused_lost_services = self._zeroconf_scan()
                for service in new_services:
                    (ip, port) = _resolve_address(service)
                    rospy.loginfo("Gateway : discovered hub via zeroconf [%s:%s]" % (str(ip), str(port)))
                    self.discovery_update_hook(ip, port)
            # Direct scanning
            discovered_hub_uris = self._direct_scan()
            for hub_uri in discovered_hub_uris:
                self._direct_hub_uri_list[:] = [uri for uri in self._direct_hub_uri_list if hub_uri != uri]
                o = urlparse(hub_uri)
                rospy.loginfo("Gateway : discovered hub directly [%s]" % hub_uri)
                self.discovery_update_hook(o.hostname, o.port)
            if not self._zeroconf_services_available and not self._direct_hub_uri_list:
                break  # nothing left to do
            rospy.sleep(self._internal_sleep_period)

    #############################
    # Private methods
    #############################

    def _direct_scan(self):
        '''
          Ping the list of hubs we are directly looking for to see if they are alive.
        '''
        discovered_hubs = []
        for uri in self._direct_hub_uri_list:
            o = urlparse(uri)
            if hub_api.ping_hub(o.hostname, o.port):
                discovered_hubs.append(uri)
        return discovered_hubs

    def _zeroconf_scan(self):
        '''
          This checks for new services and adds them. I'm not taking any
          action when a discovered service disappears yet though. Probably
          should take of that at some time.
        '''
        #rospy.loginfo("Gateway : checking for autodiscovered gateway hubs")
        try:
            response = self._list_discovered_services(self._discovery_request)
        except rospy.service.ServiceException:
            # means we've shut down, just return so it can cleanly shutdown back in run()
            return
        difference = lambda l1,l2: [x for x in l1 if x not in l2]
        new_services = difference(response.services, self._zeroconf_discovered_hubs)
        lost_services = difference(self._zeroconf_discovered_hubs, response.services)
        self._zeroconf_discovered_hubs = response.services
        #self._zeroconf_discovered_hubs.extend(new_services)
        return new_services, lost_services


###############################################################################
# Functions
###############################################################################

def _resolve_address(msg):
    '''
      Resolves a zeroconf address into ip/port portions.
      @var msg : zeroconf_msgs.DiscoveredService
      @return (string,int) : ip, port pair.
    '''
    ip = "localhost"
    if not msg.is_local:
        ip = msg.ipv4_addresses[0]
    return (ip, msg.port)

def _zeroconf_services_available():
    '''
      Check for zeroconf services on startup. If none is found within a suitable
      timeout, disable this module.
    '''
    zeroconf_timeout = 5  # Amount of time to wait for the zeroconf services to appear
    rospy.loginfo("Gateway : checking if zeroconf services are available...")
    try:
        rospy.wait_for_service("zeroconf/add_listener", timeout=zeroconf_timeout)
    except rospy.ROSException:
        rospy.logwarn("Gateway : timed out waiting for zeroconf services to become available.")
        return False
    return True

def _add_listener():
    '''
      Looks for the zeroconf services and attempts to add a rocon hub listener.
      Make sure this is called only after _zeroconf_services_available returns true.
    '''
    try:
        add_listener = rospy.ServiceProxy("zeroconf/add_listener", zeroconf_srvs.AddListener)
        if not add_listener(service_type=HubDiscovery.gateway_hub_service):
            return False
    except rospy.ROSException:
        rospy.logwarn("Gateway : timed out waiting for zeroconf services to become available.")
        return False
    return True
