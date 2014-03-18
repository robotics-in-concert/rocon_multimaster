#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import httplib
import rospy
import time

##############################################################################
# Watcher
##############################################################################


class WatcherThread(object):

    '''
      This used to be on a thread of its own, but now moved into
      the gateway's main thread for running.
    '''

    def __init__(self, gateway, watch_loop_period):
        self.trigger_update = False
        self._trigger_shutdown = False
        self._gateway = gateway
        self._master = gateway.master
        self._hub_manager = gateway.hub_manager
        self._hubs = self._hub_manager.hubs
        self._flipped_interface = gateway.flipped_interface
        self._pulled_interface = gateway.pulled_interface
        self._default_watch_loop_period = watch_loop_period
        self._watch_loop_period = watch_loop_period
        self._last_loop_timestamp = time.time()
        self._internal_sleep_period = 0.2  # 200ms

    def set_watch_loop_period(self, period):
        '''
          This is used via the gateway node service to configure the rate of the
          watcher thread. If not positive, it will reset to the default.

          @param period : new setting in seconds
          @type float
        '''
        self._watch_loop_period = self._default_watch_loop_period if period <= 0.0 else period

    def get_watch_loop_period(self):
        '''
          Use Duration's to_sec() method to convert this to float.

          @return the watcher loop period.
          @rtype float
        '''
        return self._watch_loop_period

    def start(self):
        '''
          The watcher thread - monitors both the local master's system state (list of connections)
          and the various rules to make sure rules and existing connections or flips are in sync.
        '''
        while not rospy.is_shutdown():
            # don't waste time processing if we're not connected to at least one hub
            if self._gateway.is_connected():
                try:
                    connections = self._master.get_connection_state()
                except httplib.ResponseNotReady:
                    rospy.logwarn("Gateway : received 'ResponseNotReady' from master api")
                    self._sleep()
                    continue
                remote_gateway_hub_index = self._hub_manager.create_remote_gateway_hub_index()
                self._gateway.update_network_information()
                self._gateway.update_flipped_interface(connections, remote_gateway_hub_index)
                self._gateway.update_public_interface(connections)
                self._gateway.update_pulled_interface(connections, remote_gateway_hub_index)
                registrations = self._hub_manager.get_flip_requests()
                self._gateway.update_flipped_in_interface(registrations, remote_gateway_hub_index)
            self._sleep()

    def _sleep(self):
        '''
          Internal non-interruptible sleep loop to check for shutdown and update triggers.
          This lets us set a really long watch_loop update if we wish.
        '''
        while (not rospy.is_shutdown() and not self.trigger_update and
               (time.time() - self._last_loop_timestamp < self._watch_loop_period)):
            rospy.rostime.wallsleep(self._internal_sleep_period)
        self.trigger_update = False
        self._last_loop_timestamp = time.time()
