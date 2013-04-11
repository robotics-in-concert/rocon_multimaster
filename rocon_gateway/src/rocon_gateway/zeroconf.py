#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway/LICENSE
#

###############################################################################
# Imports
###############################################################################

import threading
import rospy
import zeroconf_msgs.srv

###############################################################################
# Constants
###############################################################################

gateway_hub_service = "_ros-multimaster-hub._tcp"

###############################################################################
# Thread
###############################################################################


class HubDiscovery(threading.Thread):
    '''
      Used to discover hubs via zeroconf.
    '''
    def __init__(self, external_discovery_update_hook):
        threading.Thread.__init__(self)
        self.discovery_update_hook = external_discovery_update_hook
        self._trigger_shutdown = False
        self._services = setup_ros_services()
        self._discovery_request = zeroconf_msgs.srv.ListDiscoveredServicesRequest()
        self._discovery_request.service_type = gateway_hub_service
        self._discovered_hubs = []
        if self._services:
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
            new_services = self._scan_for_zeroconf_hubs()
            if new_services:
                self.discovery_update_hook(new_services)
            rospy.sleep(self._internal_sleep_period)

    def _scan_for_zeroconf_hubs(self):
        '''
          This checks for new services and adds them. I'm not taking any
          action when a discovered service disappears yet though. Probably
          should take of that at some time.
        '''
        #rospy.loginfo("Gateway : checking for autodiscovered gateway hubs")
        response = self._services["list_discovered_services"](self._discovery_request)
        difference = lambda l1,l2: [x for x in l1 if x not in l2]
        new_services = difference(response.services, self._discovered_hubs)
        self._discovered_hubs.extend(new_services)
        return new_services


###############################################################################
# Functions
###############################################################################


def resolve_address(msg):
    '''
      Resolves a zeroconf address into ip/port portions.
      @var msg : zeroconf_msgs.DiscoveredService
      @return (string,int) : ip, port pair.
    '''
    ip = "localhost"
    if not msg.is_local:
        ip = msg.ipv4_addresses[0]
    return (ip, msg.port)


def setup_ros_services():
    '''
      Looks to see if it can find the zeroconf services that
      will help it auto-discover a hub. If it finds them,
      it hooks up the required ros services with the zeroconf node.

      @return success of the hookup
      @rtype bool
    '''
    zeroconf_services = {}
    zeroconf_timeout = 5  # Amount of time to wait for the zeroconf services to appear
    rospy.loginfo("Gateway : checking if zeroconf services are available...")
    try:
        rospy.wait_for_service("zeroconf/add_listener", timeout=zeroconf_timeout)
        zeroconf_services["add_listener"] = rospy.ServiceProxy("zeroconf/add_listener", zeroconf_msgs.srv.AddListener)
        zeroconf_services["list_discovered_services"] = rospy.ServiceProxy("zeroconf/list_discovered_services", zeroconf_msgs.srv.ListDiscoveredServices)
        if not zeroconf_services["add_listener"](service_type=gateway_hub_service):
            zeroconf_services = {}  # failure
    except rospy.ROSException:
        rospy.logwarn("Gateway : timed out waiting for zeroconf services to become available.")
    return zeroconf_services
