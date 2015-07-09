#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#
###############################################################################
# Imports
###############################################################################

import threading
from urlparse import urlparse
import rospy
import time
import zeroconf_msgs.srv as zeroconf_srvs
from gateway_msgs.msg import ErrorCodes

from . import hub_client

###############################################################################
# Thread
###############################################################################


class HubDiscovery(threading.Thread):

    gateway_hub_service = "_ros-multimaster-hub._tcp"

    '''
      Used to discover hubs via zeroconf.
    '''
    def __init__(self, external_discovery_update_hook, direct_hub_uri_list=[], disable_zeroconf=False, blacklisted_hubs={}):
        '''
          :param external_discovery_update: is a callback function that takes action on a discovery
          :type external_discovery_update: GatewayNode.register_gateway(ip, port)

          :param str[] direct_hub_uri_list: list of uri's to hubs (e.g. http://localhost:6380)

          :param disallowed_hubs:
          :type disallowed_hubs: # 'ip:port' : (error_code, error_code_str) dictionary of hubs that have been blacklisted (maintained by manager of this class)
        '''
        threading.Thread.__init__(self)
        self.discovery_update_hook = external_discovery_update_hook
        self._trigger_shutdown = False
        self.trigger_update = False
        self._direct_hub_uri_list = direct_hub_uri_list
        self._direct_discovered_hubs = []
        self._zeroconf_services_available = False if disable_zeroconf else _zeroconf_services_available()
        self._blacklisted_hubs = blacklisted_hubs
        if self._zeroconf_services_available:
            self._discovery_request = zeroconf_srvs.ListDiscoveredServicesRequest()
            self._discovery_request.service_type = HubDiscovery.gateway_hub_service
            _add_listener()
            self._list_discovered_services = rospy.ServiceProxy("zeroconf/list_discovered_services", zeroconf_srvs.ListDiscoveredServices, persistent=True)
            self._zeroconf_discovered_hubs = []
        self._discovered_hubs_modification_mutex = threading.Lock()
        # Only run the thread if we need to.
        if self._zeroconf_services_available or self._direct_hub_uri_list:
            self.start()

    def shutdown(self):
        '''
          Called from the main program to shutdown this thread.
        '''
        self._trigger_shutdown = True
        self._trigger_update = True  # causes it to interrupt a sleep and drop back to check shutdown condition
        if self.is_alive():  # python complains if you join a non-started thread
            self.join()  # wait for the thread to finish

    def run(self):
        '''
          The hub discovery thread worker function. Monitors zeroconf for the presence of new hubs.

          We spin fast initially for convenience, and then wind down once we've detected
          a hub.

          Note that the zeroconf service is persistent. Alternatively we could use the zeroconf
          subscriber to be a wee bit more efficient.
        '''
        half_sec = 0.5  # rospy.Duration(0, 500000000)
        self._loop_period = half_sec
        self._internal_sleep_period = half_sec
        self._last_loop_timestamp = time.time()  # rospy.Time.now()
        # error codes which inform the client is should stop scanning for this hub
        reasons_not_to_keep_scanning = [
            ErrorCodes.SUCCESS,
            ErrorCodes.HUB_CONNECTION_ALREADY_EXISTS,
            ErrorCodes.HUB_CONNECTION_NOT_IN_NONEMPTY_WHITELIST,
            # this now has to be permitted as we will often have zeroconf failing for gateways
            # that have dropped out of wireless range.
            # ErrorCodes.HUB_CONNECTION_UNRESOLVABLE
        ]
        unresolvable_hub = []
        while not rospy.is_shutdown() and not self._trigger_shutdown:
            self._discovered_hubs_modification_mutex.acquire()
            # Zeroconf scanning
            if self._zeroconf_services_available:
                new_services, unused_lost_services = self._zeroconf_scan()
                for service in new_services:
                    (ip, port) = _resolve_address(service)
                    service_uri = str(ip) + ':' + str(port)
                    if service_uri not in self._blacklisted_hubs.keys():
                        result, reason = self.discovery_update_hook(ip, port)
                        if result == ErrorCodes.HUB_CONNECTION_UNRESOLVABLE:
                            if service_uri not in unresolvable_hub:
                                rospy.loginfo("Gateway : unresolvable hub [%s]" % reason)
                                unresolvable_hub.append(service_uri)
                        elif result == ErrorCodes.SUCCESS:
                            # we're good
                            rospy.loginfo("Gateway : discovered hub via zeroconf [%s:%s]" % (str(ip), str(port)))
                            if service_uri in unresolvable_hub:
                                unresolvable_hub.remove(service_uri)
                        else:  # any of the other reasons not to keep scanning
                            rospy.loginfo("Gateway : blacklisting hub [%s]" % reason)
                            self._zeroconf_discovered_hubs.append(service)
            # Direct scanning
            new_hubs, unused_lost_hubs = self._direct_scan()
            for hub_uri in new_hubs:
                hostname, port = _resolve_url(hub_uri)
                rospy.loginfo("Gateway : discovered hub directly [%s]" % hub_uri)
                result, _ = self.discovery_update_hook(hostname, port)
                if result in reasons_not_to_keep_scanning:
                    self._direct_discovered_hubs.append(hub_uri)
            self._discovered_hubs_modification_mutex.release()
            if not self._zeroconf_services_available and not self._direct_hub_uri_list:
                rospy.logfatal("Gateway : zeroconf unavailable and no valid direct hub uris. Stopping hub discovery.")
                break  # nothing left to do
            self._sleep()
        if self._zeroconf_services_available:
            self._list_discovered_services.close()

    def disengage_hub(self, hub):
        '''
          Called when a discovered hub is lost in the upstream application.

          This method should remove the hub from the list of discovered hubs.
          When the hub comes back up again, the hub discovery thread will
          call the discovery_update_hook again

          @param hub: hub to be disengage
          @type Hub
        '''
        self._discovered_hubs_modification_mutex.acquire()
        self._direct_discovered_hubs[:] = [x for x in self._direct_discovered_hubs
                                           if not _match_url_to_hub_url(x, hub.uri)]
        if self._zeroconf_services_available:
            self._zeroconf_discovered_hubs[:] = [x for x in self._zeroconf_discovered_hubs
                                                 if not _match_zeroconf_address_to_hub_url(x, hub.uri)]
        self._discovered_hubs_modification_mutex.release()

    def _sleep(self):
        '''
          Internal non-interruptible sleep loop to check for shutdown and update triggers.
          This lets us set a really long watch_loop update if we wish.
        '''
        while not rospy.is_shutdown() and not self.trigger_update and (time.time() - self._last_loop_timestamp < self._loop_period):
            rospy.rostime.wallsleep(self._internal_sleep_period)
        self.trigger_update = False
        self._last_loop_timestamp = time.time()

    #############################
    # Private methods
    #############################

    def _direct_scan(self):
        '''
          Ping the list of hubs we are directly looking for to see if they are alive.
        '''
        discovered_hubs = []
        remove_uris = []
        for uri in self._direct_hub_uri_list:
            (hostname, port) = _resolve_url(uri)
            if not hostname:
                rospy.logerr("Gateway : Unable to parse direct hub uri [%s]" % uri)
                remove_uris.append(uri)
                continue
            (ping_result, unused_ping_error_message) = hub_client.ping_hub(hostname, port)
            if ping_result:
                discovered_hubs.append(uri)
        difference = lambda l1, l2: [x for x in l1 if x not in l2]
        self._direct_hub_uri_list[:] = difference(self._direct_hub_uri_list, remove_uris)
        new_hubs = difference(discovered_hubs, self._direct_discovered_hubs)
        lost_hubs = difference(self._direct_discovered_hubs, discovered_hubs)
        # self._direct_discovered_hubs = discovered_hubs
        # self._direct_discovered_hubs.extend(discovered_hubs)
        return new_hubs, lost_hubs

    def _zeroconf_scan(self):
        '''
          This checks for new services and adds them. I'm not taking any
          action when a discovered service disappears yet though. Probably
          should take of that at some time.
        '''
        # rospy.loginfo("Gateway : checking for autodiscovered gateway hubs")
        try:
            response = self._list_discovered_services(self._discovery_request)
        except (rospy.service.ServiceException, rospy.exceptions.ROSInterruptException):
            # means we've shut down, just return so it can cleanly shutdown back in run()
            return [], []
        except (rospy.exceptions.TransportTerminated, AttributeError) as unused_e:
            # We should never ever see this - but we're calling the zeroconf node
            # when we may be in a shutdown hook. Instead of getting one of the standard
            # exceptions above, it gives us anyone of these curious errors.
            # Rospy could handle this better...
            return [], []
        difference = lambda l1, l2: [x for x in l1 if x not in l2]
        new_services = difference(response.services, self._zeroconf_discovered_hubs)
        lost_services = difference(self._zeroconf_discovered_hubs, response.services)
        return new_services, lost_services

###############################################################################
# Functions
###############################################################################


def _resolve_url(url):
    '''
      Resolved a url into ip/port portions using urlparse
      @var url : The url to parse (may or may not have a scheme)
      @return (string,int) : ip, port pair
    '''
    o = urlparse(url)
    ip = None
    port = None
    try:
        if o.hostname is not None and o.port is not None:
            ip, port = str(o.hostname), int(o.port)
        else:
            # Explicit attempt to parse hostname:port
            values = url.split(':')
            if len(values) == 2:
                ip, port = str(values[0]), int(values[1])
    except ValueError:
        ip, port = None, None
    return ip, port


def _match_url_to_hub_url(url, hub_uri):
    '''
      @param url: The original url used to specify the hub
      @type string

      @param hub_uri: The uri constructed by the hub, devoid of any URL scheme
      @type string: of the form ip:port
    '''
    (ip, port) = _resolve_url(url)
    return (hub_uri == str(ip) + ":" + str(port))


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


def _match_zeroconf_address_to_hub_url(msg, hub_uri):
    '''
      @param msg: The original zeroconf address used to specify the hub
      @type zeroconf_msgs.DiscoveredService

      @param hub_uri: The uri constructed by the hub, devoid of any URL scheme
      @type string: of the form ip:port
    '''
    (ip, port) = _resolve_address(msg)
    return (hub_uri == str(ip) + ":" + str(port))


def _zeroconf_services_available():
    '''
      Check for zeroconf services on startup. If none is found within a suitable
      timeout, disable this module.
    '''
    zeroconf_timeout = 15  # Amount of time to wait for the zeroconf services to appear
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
    except rospy.ServiceException:
        rospy.logwarn("Gateway : unable to connect to zeroconf/add_listener service [timeout||crashed]].")
        return False
    return True
