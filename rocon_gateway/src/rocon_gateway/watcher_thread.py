#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway/LICENSE 
#
##############################################################################
# Imports
##############################################################################

import roslib
roslib.load_manifest('rocon_gateway')
import rospy
import threading
import httplib

from gateway_msgs.msg import Rule, ConnectionType

##############################################################################
# Watcher
##############################################################################


class WatcherThread(threading.Thread):
    '''
    '''

    def __init__(self, gateway, watch_loop_period):
        threading.Thread.__init__(self)
        self.trigger_update = False
        self._trigger_shutdown = False
        self._gateway = gateway
        self._master = gateway.master
        self._hub = gateway.hub
        #self._public_interface = gateway.public_interface
        self._flipped_interface = gateway.flipped_interface
        self._pulled_interface = gateway.pulled_interface
        self._watch_loop_period = rospy.Duration(watch_loop_period)
        self._last_loop_timestamp = rospy.Time.now()
        self._internal_sleep_period = rospy.Duration(0, 200000000)  # 200ms
        self.start()

    def shutdown(self):
        '''
          Called from the main program to shutdown this thread.
        '''
        self._trigger_shutdown = True
        self._trigger_update = True  # causes it to interrupt a sleep and drop back to check shutdown condition
        self.join()  # wait for the thread to finish

    def run(self):
        '''
          The watcher thread - monitors both the local master's system state (list of connections)
          and the various rules to make sure rules and existing connections or flips are in sync.
        '''
        while not rospy.is_shutdown() and not self._trigger_shutdown:
            if self._gateway.is_connected:
                try:
                    connections = self._master.getConnectionState()
                except httplib.ResponseNotReady:
                    rospy.logwarn("Gateway : received 'ResponseNotReady' from master api")
                    self._sleep()
                    continue
                gateways = self._hub.listRemoteGatewayNames()
                self._gateway.update_flip_interface(connections, gateways)
                self._gateway.update_public_interface(connections)
                self._gateway.update_pulled_interface(connections, gateways)

            self._sleep()

    def _sleep(self):
        '''
          Internal interruptible sleep loop to check for shutdown and update triggers.
          This lets us set a really long watch_loop update if we wish.
        '''
        while not rospy.is_shutdown() and not self.trigger_update and (rospy.Time.now() - self._last_loop_timestamp < self._watch_loop_period):
            rospy.sleep(self._internal_sleep_period)
        self.trigger_update = False
        self._last_loop_timestamp = rospy.Time.now()
