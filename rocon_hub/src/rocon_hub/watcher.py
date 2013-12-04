#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_server/rocon_hub/LICENSE
#

##############################################################################
# Imports
##############################################################################

from gateway_msgs.msg import ConnectionStatistics
from rocon_gateway import gateway_hub
from rocon_hub_client import hub_api
from rocon_utilities import WallRate


import rocon_hub_client
import rospy
import sys
import threading

##############################################################################
# Main watcher thread
##############################################################################

class WatcherThread(threading.Thread):

    def __init__(self, ip, port):
        threading.Thread.__init__(self)
        self.daemon = True
        self.gateway_unavailable_timeout = \
                rospy.get_param('~gateway_unavailable_timeout', 30.0)
        self.gateway_dead_timeout = \
                rospy.get_param('~gateway_dead_timeout', 7200.0)
        self.gateway_ping_frequency = \
                rospy.get_param('~gateway_ping_frequency', 0.2)
        self.watcher_thread_rate = rospy.get_param('~watcher_thread_rate', 0.2)
        try:
            self.hub = gateway_hub.GatewayHub(ip, port, [], [])
        except rocon_hub_client.HubError as e:
            rospy.logfatal("HubWatcher: Unable to connect to hub: %s" % str(e))
            sys.exit(-1)
        self.unavailable_gateways = []

    def run(self):
        '''
          Run the hub watcher (sidekick) thread at the rate specified by the
          watcher_thread_rate parameter. The wathcer thread does the following:
              1. For all gateways available, see if we have a pinger available.
              2. Add and remove pingers as necessary
              3. Depending on pinger stats, update hub appropriately
        '''
        rate = WallRate(self.watcher_thread_rate)
        while True:
            remote_gateway_names = self.hub.list_remote_gateway_names()
            
            # Check all pingers
            for name in remote_gateway_names:

                gateway_key = hub_api.create_rocon_key(name)
                # Get time for this gateway when hub was last seen
                ping_key = hub_api.create_rocon_gateway_key(name, ':ping')
                expiration_time = self.hub._redis_server.ttl(ping_key)

                seconds_since_last_seen = \
                        ConnectionStatistics.MAX_TTL - expiration_time
                # Check if gateway gone for low timeout (unavailable)
                if seconds_since_last_seen > self.gateway_unavailable_timeout:
                    if name not in self.unavailable_gateways:
                        rospy.logwarn("HubWatcherThread: Gateway " + name +
                                      " has been unavailable for " +
                                      str(self.gateway_unavailable_timeout) +
                                      " seconds! Marking as unavailable.")
                        self.unavailable_gateways.append(name)
                    self.hub.mark_named_gateway_available(gateway_key, False,
                             seconds_since_last_seen)
                else:
                    if name in self.unavailable_gateways:
                        self.unavailable_gateways.remove(name)
                    self.hub.mark_named_gateway_available(gateway_key, True,
                             seconds_since_last_seen)

                # Check if gateway gone for high timeout (dead)
                if seconds_since_last_seen > self.gateway_dead_timeout:
                    rospy.logwarn("HubWatcherThread: Gateway " + name +
                                  " has been unavailable for " +
                                  str(self.gateway_dead_timeout) +
                                  " seconds! Removing from hub.")
                    self.hub.unregister_named_gateway(gateway_key)
            rate.sleep()
