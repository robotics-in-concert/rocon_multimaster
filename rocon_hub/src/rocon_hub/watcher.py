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

    def __init__(self, ip, ping_frequency = 0.2, timeout = 30.0):

        threading.Thread.__init__(self)
        self.daemon = True

        self.ip = ip
        self.ping_frequency = ping_frequency
        self.time_last_seen = time.time()
        self.timeout = timeout

        # Format is min, avg, max, mean deviation
        self.latency_stats = [0.0, 0.0, 0.0, 0.0]

    def is_alive(self):
        return time.time() - self.time_last_seen <= self.timeout

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
                output = subprocess.call("ping -c 5 -i 0.2 %s" % self.ip,
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
        self.gateway_timeout = rospy.get_param('~gateway_timeout', 30.0)
        self.gateway_ping_frequency = rospy.get_param('~gateway_ping_frequency', 0.2)
        self.watcher_thread_rate = rospy.get_param('~watcher_thread_rate', 0.2)
        try:
            self.hub = gateway_hub.GatewayHub(ip, port, [], [])
        except rocon_hub_client.HubError as e:
            rospy.logfatal("HubWatcherThread: Unable to connect to hub: %s"%str(e))
            sys.exit(-1)
        self.pingers = {}

    def run(self):
        rate = WallRate(self.watcher_thread_rate) 
        while True:
            remote_gateway_names = self.hub.list_remote_gateway_names()
            new_gateways = [x for x in remote_gateway_names 
                            if x not in self.pingers]  
            for gateway in new_gateways:
                gateway_info = self.hub.remote_gateway_info(gateway)
                self.pingers[gateway] = Pinger(gateway_info.ip, self.gateway_ping_frequency, self.gateway_timeout)
                self.pingers[gateway].start()
            remove_pingers = [x for x in self.pingers 
                              if x not in remote_gateway_names]  
            for pinger in remove_pingers:
                del self.pingers[pinger]
            
            # Check all pingers
            for name, pinger in self.pingers.iteritems():
                if not pinger.is_alive():
                    rospy.logwarn("HubWatcherThread: " + name + 
                                  " has gone down! Removing from hub.")
                    gateway_key = hub_api.create_rocon_key(name)
                    self.hub._unregister_named_gateway(gateway_key)
            rate.sleep()

