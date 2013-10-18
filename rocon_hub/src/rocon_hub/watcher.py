#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_server/rocon_hub/LICENSE
#

##############################################################################
# Imports
##############################################################################

from rocon_gateway import gateway_hub
import rocon_hub_client
import rospy
import subprocess
import sys
import threading
import time

class Pinger(threading.Thread):
    def __init__(self, ip):
        threading.Thread.__init__(self)
        self.daemon = True
        self.ip = ip
        self.failed_count = 0

    def is_alive(self):
        return self.failed_count < 3

    def run(self):
        while True:
            ret = subprocess.call("ping -c 1 %s" % self.ip,
                                  shell=True,
                                  stdout=open('/dev/null', 'w'),
                                  stderr=subprocess.STDOUT)
            if ret == 0:
                self.failed_count = 0
            else:
                self.failed_count += 1
            time.sleep(1.0)

class WatcherThread(threading.Thread):

    def __init__(self, ip, port):
        threading.Thread.__init__(self)
        self.daemon = True
        try:
            self.hub = gateway_hub.GatewayHub(ip, port, [], [])
        except rocon_hub_client.HubError as e:
            rospy.logfatal("WatcherThread: Unable to connect to hub: %s"%str(e))
            sys.exit(-1)
        self.pingers = {}

    def run(self):
        while True:
            remote_gateway_names = self.hub.list_remote_gateway_names()
            new_gateways = [x for x in remote_gateway_names if x not in self.pingers]  
            for gateway in new_gateways:
                gateway_info = self.hub.remote_gateway_info(gateway)
                self.pingers[gateway] = Pinger(gateway_info.ip)
                self.pingers[gateway].start()
            remove_pingers = [x for x in self.pingers if x not in remote_gateway_names]  
            for pinger in remove_pingers:
                del self.pingers[pinger]
            
            # Check all pingers
            for name, pinger in self.pingers.iteritems():
                if pinger.is_alive():
                    rospy.loginfo(name + " is alive!")
                else:
                    rospy.logwarn(name + " is NOT alive! Should drop all information from the hub.")
            time.sleep(0.5)

