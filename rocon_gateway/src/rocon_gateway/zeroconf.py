#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/hydro-devel/rocon_gateway/LICENSE
#

###############################################################################
# Imports
###############################################################################

import threading
import rospy
import zeroconf_msgs.srv as zeroconf_srvs

###############################################################################
# Thread
###############################################################################


class HubDiscovery(threading.Thread):

    gateway_hub_service = "_ros-multimaster-hub._tcp"

    '''
      Used to discover hubs via zeroconf.
    '''
    def __init__(self, external_discovery_update_hook):
        threading.Thread.__init__(self)
        self.discovery_update_hook = external_discovery_update_hook
        self._trigger_shutdown = False
        if _zeroconf_services_available():
            self._discovery_request = zeroconf_srvs.ListDiscoveredServicesRequest()
            self._discovery_request.service_type = HubDiscovery.gateway_hub_service
            _add_listener()
            self._list_discovered_services = rospy.ServiceProxy("zeroconf/list_discovered_services", zeroconf_srvs.ListDiscoveredServices)
            self._discovered_hubs = []
            self.start()

    def shutdown(self):
        '''
          Called from the main program to shutdown this thread.
        '''
        self._trigger_shutdown = True
        self.join()  # wait for the thread to finish

    def run(self):
        '''
          The hub discovery thread worker function. Monitors zeroconf for the presence of new hubs.
        '''
        self._internal_sleep_period = rospy.Duration(0, 200000000)  # 200ms
        while not rospy.is_shutdown() and not self._trigger_shutdown:
            new_services = self._scan()
            for service in new_services:
                (ip, port) = _resolve_address(service)
                rospy.loginfo("Gateway : discovered hub via zeroconf at " + str(ip) + ":" + str(port))
                self.discovery_update_hook(ip, port)
            rospy.sleep(self._internal_sleep_period)

    #############################
    # Private methods
    #############################

    def _scan(self):
        '''
          This checks for new services and adds them. I'm not taking any
          action when a discovered service disappears yet though. Probably
          should take of that at some time.
        '''
        #rospy.loginfo("Gateway : checking for autodiscovered gateway hubs")
        response = self._list_discovered_services(self._discovery_request)
        difference = lambda l1,l2: [x for x in l1 if x not in l2]
        new_services = difference(response.services, self._discovered_hubs)
        self._discovered_hubs.extend(new_services)
        return new_services

###############################################################################
# Internal Methods
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
    '''
    try:
        add_listener = rospy.ServiceProxy("zeroconf/add_listener", zeroconf_srvs.AddListener)
        if not add_listener(service_type=HubDiscovery.gateway_hub_service):
            return False
    except rospy.ROSException:
        rospy.logwarn("Gateway : timed out waiting for zeroconf services to become available.")
        return False
    return True
