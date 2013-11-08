#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_server/rocon_hub/LICENSE
#

##############################################################################
# Imports
##############################################################################

from rocon_gateway import gateway_hub
from rocon_hub_client import hub_api
from rocon_utilities import WallRate

import rocon_hub_client
import rospy
import subprocess
import sys
import threading
import time

##############################################################################
# Helpers
##############################################################################

class Pinger(threading.Thread):

    def __init__(self, ip, ping_frequency = 0.2, 
                 unavailable_timeout = 30.0, dead_timeout = 7200.0):

        threading.Thread.__init__(self)
        self.daemon = True

        self.ip = ip
        self.ping_frequency = ping_frequency
        self.time_last_seen = time.time()
        self.unavailable_timeout = unavailable_timeout
        self.dead_timeout = dead_timeout

        # Format is min, avg, max, mean deviation
        self.latency_stats = [0.0, 0.0, 0.0, 0.0]

    def is_unavailable(self):
        return time.time() - self.time_last_seen > self.unavailable_timeout
    
    def is_dead(self):
        return time.time() - self.time_last_seen > self.dead_timeout

    def get_time_since_last_seen(self):
        return time.time() - self.time_last_seen

    def get_latency(self):
        '''
          Latency states are returned as list of 4 values
          [min,avg,max,mean deviation]
        '''
        return self.latency_stats

    def run(self):
        rate = WallRate(self.ping_frequency)
        while True:
            # In case of failure, this call will take approx 10s
            try:
                # Send 5 pings at an interval of 0.2s
                output = subprocess.check_output("ping -c 5 -i 0.2 %s" % self.ip,
                                         shell=True, stderr=subprocess.STDOUT)
                self.time_last_seen = time.time()
                self.latency_stats = [float(x) for x in output.splitlines()[-1].split(' ')[-2].split('/')]
            except subprocess.CalledProcessError:
                # Ping failed. Do not update time last seen
                pass
            rate.sleep()

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
        self.gateway_ping_frequency = rospy.get_param('~gateway_ping_frequency', 0.2)
        self.watcher_thread_rate = rospy.get_param('~watcher_thread_rate', 0.2)
        try:
            self.hub = gateway_hub.GatewayHub(ip, port, [], [])
        except rocon_hub_client.HubError as e:
            rospy.logfatal("HubWatcherThread: Unable to connect to hub: %s"%str(e))
            sys.exit(-1)
        self.pingers = {}
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

            # Add new pingers
            new_gateways = [x for x in remote_gateway_names 
                            if x not in self.pingers]  
            for gateway in new_gateways:
                gateway_info = self.hub.remote_gateway_info(gateway)
                self.pingers[gateway] = Pinger(gateway_info.ip, 
                                               self.gateway_ping_frequency, 
                                               self.gateway_unavailable_timeout,
                                               self.gateway_dead_timeout)
                self.pingers[gateway].start()
            remove_pingers = [x for x in self.pingers 
                              if x not in remote_gateway_names]  
            for pinger in remove_pingers:
                del self.pingers[pinger]
            
            # Check all pingers
            for name, pinger in self.pingers.iteritems():
                gateway_key = hub_api.create_rocon_key(name)

                # Check if gateway gone for low timeout
                if pinger.is_unavailable():
                    if name not in self.unavailable_gateways:
                        rospy.logwarn("HubWatcherThread: Gateway " + name + 
                                      " has been unavailable for " + 
                                      str(self.gateway_unavailable_timeout) +
                                      " seconds! Marking as unavailable.")
                        self.hub.mark_named_gateway_available(gateway_key,
                                 False, pinger.get_time_since_last_seen())
                        self.unavailable_gateways.append(name)
                else:
                    if name in self.unavailable_gateways:
                        self.unavailable_gateways.remove(name)
                    self.hub.update_named_gateway_latency_stats(name, 
                             pinger.get_latency())
                    self.hub.mark_named_gateway_available(gateway_key, True)

                # Check if gateway gone for high timeout
                if pinger.is_dead():
                    rospy.logwarn("HubWatcherThread: Gateway " + name + 
                                  " has been unavailable for " + 
                                  str(self.gateway_dead_timeout) +
                                  " seconds! Removing from hub.")
                    # This should automatically remove the pinger in the next
                    # iteration
                    self.hub.unregister_named_gateway(gateway_key)
            rate.sleep()
