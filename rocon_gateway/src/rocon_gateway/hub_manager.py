#!/usr/bin/env pythonupdate
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#
###############################################################################
# Imports
###############################################################################

import threading

import rospy
import gateway_msgs.msg as gateway_msgs
import rocon_hub_client

from .exceptions import GatewayUnavailableError
from . import gateway_hub
from . import utils

##############################################################################
# Hub Manager
##############################################################################


class HubManager(object):

    ##########################################################################
    # Init & Shutdown
    ##########################################################################

    def __init__(self, hub_whitelist, hub_blacklist):
        self._param = {}
        self._param['hub_whitelist'] = hub_whitelist
        self._param['hub_blacklist'] = hub_blacklist
        self.hubs = []
        self._hub_lock = threading.Lock()

    def shutdown(self):
        for hub in self.hubs:
            hub.unregister_gateway()

    def is_connected(self):
        return True if self.hubs else False

    ##########################################################################
    # Introspection
    ##########################################################################

    def list_remote_gateway_names(self):
        '''
          Parse all the hubs and retrieve the list of remote gateway names.

          Note: not sure where is most convenient, here or in gateway class.

          @return list of remote gateway names (with hashes), e.g. gateway345ae2c...
          @rtype list of str
        '''
        remote_gateway_names = []
        self._hub_lock.acquire()
        for hub in self.hubs:
            remote_gateway_names.extend(hub.list_remote_gateway_names())
        self._hub_lock.release()
        # return the list without duplicates
        return list(set(remote_gateway_names))

    def create_remote_gateway_hub_index(self):
        '''
          Utility function to parse all hubs for the remote gateways and
          create a dictionary of the type:

            dic['remote_gateway_name'] = ['hub1', 'hub2']

          where the hub list is a list of actual hub object references.
        '''
        dic = {}
        self._hub_lock.acquire()
        for hub in self.hubs:
            for remote_gateway in hub.list_remote_gateway_names():
                if remote_gateway in dic:
                    dic[remote_gateway].append(hub)
                else:
                    dic[remote_gateway] = [hub]
        self._hub_lock.release()
        return dic

    def get_flip_requests(self):
        '''
          Returns all unblocked flip requests received by this hub

          @return list of flip registration requests
          @rtype list of utils.Registration
        '''
        registrations = []
        self._hub_lock.acquire()
        for hub in self.hubs:
            registrations.extend(hub.get_unblocked_flipped_in_connections())
        self._hub_lock.release()
        return registrations

    def remote_gateway_info(self, remote_gateway_name):
        '''
          Return information that a remote gateway has posted on the hub(s).

          @param remote_gateway_name : the hash name for the remote gateway
          @type str

          @return remote gateway information
          @rtype gateway_msgs.RemotGateway or None
        '''
        remote_gateway_info = None
        self._hub_lock.acquire()
        for hub in self.hubs:
            if remote_gateway_name in hub.list_remote_gateway_names():
                # I don't think we need more than one hub's info....
                remote_gateway_info = hub.remote_gateway_info(remote_gateway_name)
                if remote_gateway_info is not None:
                    break
        self._hub_lock.release()
        return remote_gateway_info

    def get_remote_gateway_firewall_flag(self, remote_gateway_name):
        '''
          Return information that a remote gateway has posted on the hub(s).

          @param remote_gateway_name : the hash name for the remote gateway
          @type string

          @return True, false if the flag is set or not, None if remote
                  gateway information cannot found
          @rtype Bool
        '''
        firewall_flag = None
        self._hub_lock.acquire()
        for hub in self.hubs:
            if remote_gateway_name in hub.list_remote_gateway_names():
                # I don't think we need more than one hub's info....
                try:
                    firewall_flag = hub.get_remote_gateway_firewall_flag(remote_gateway_name)
                    break
                except GatewayUnavailableError:
                    pass  # cycle through the other hubs looking as well.
        self._hub_lock.release()
        return firewall_flag

    def send_unflip_request(self, remote_gateway_name, remote_rule):
        '''
          Send an unflip request to the specified gateway through all available
          hubs.

          Doesn't raise GatewayUnavailableError if nothing got sent as the higher level
          doesn't need any logic there yet (only called from gateway.shutdown).

          @param remote_gateway_name : the hash name for the remote gateway
          @type string

          @param remote_rule : the remote rule to unflip
          @type gateway_msgs.RemoteRule
        '''
        self._hub_lock.acquire()
        for hub in self.hubs:
            if remote_gateway_name in hub.list_remote_gateway_names():
                try:
                    if hub.send_unflip_request(remote_gateway_name, remote_rule):
                        self._hub_lock.release()
                        return
                except GatewayUnavailableError:
                    pass  # cycle through the other hubs looking as well.
        self._hub_lock.release()

    ##########################################################################
    # Hub Connections
    ##########################################################################

    def connect_to_hub(self,
                       ip,
                       port,
                       firewall_flag,
                       gateway_unique_name,
                       gateway_disengage_hub,  # hub connection lost hook
                       gateway_ip,
                       existing_advertisements
                       ):
        '''
          Attempts to make a connection and register the gateway with a hub.
          This is called from the gateway node's _register_gateway method.

          @param ip
          @param port
          @param firewall_flag
          @param gateway_unique_name
          @param remote_gateway_request_callbacks
          @type method : Gateway.remote_gateway_request_callbacks()
          @param gateway_disengage_hub : this is the hub connection lost hook
          @type method : Gateway.disengage_hub()
          @param gateway_ip
          @param existing advertisements
          @type { utils.ConnectionTypes : utils.Connection[] }

          @return an integer indicating error (important for the service call)
          @rtype gateway_msgs.ErrorCodes

          @raise
        '''
        try:
            new_hub = gateway_hub.GatewayHub(ip, port, self._param['hub_whitelist'], self._param['hub_blacklist'])
        except rocon_hub_client.HubError as e:
            return None, e.id, str(e)
        already_exists_error = False
        self._hub_lock.acquire()
        for hub in self.hubs:
            if hub == new_hub:
                already_exists_error = True
                break
        self._hub_lock.release()
        if not already_exists_error:
            self._hub_lock.acquire()
            new_hub.register_gateway(firewall_flag,
                                     gateway_unique_name,
                                     gateway_disengage_hub,  # hub connection lost hook
                                     gateway_ip,
                                     )
            for connection_type in utils.connection_types:
                for advertisement in existing_advertisements[connection_type]:
                    new_hub.advertise(advertisement)
            self.hubs.append(new_hub)
            self._hub_lock.release()
            return new_hub, gateway_msgs.ErrorCodes.SUCCESS, "success"
        else:
            return None, gateway_msgs.ErrorCodes.HUB_CONNECTION_ALREADY_EXISTS, "already connected to this hub"

    def disengage_hub(self, hub_to_be_disengaged):
        '''
          Disengages a hub. Make sure all necessary connections
          are cleaned up before calling this (Gateway.disengage_hub).

          @param hub_to_be_disengaged
        '''
        #uri = str(ip) + ":" + str(port)
        # Could dig in and find the name here, but not worth the bother.
        hub_to_be_disengaged.disconnect()  # necessary to kill failing socket receives
        self._hub_lock.acquire()
        if hub_to_be_disengaged in self.hubs:
            rospy.loginfo("Gateway : disengaged connection with the hub [%s][%s]" % (
                hub_to_be_disengaged.name, hub_to_be_disengaged.uri))
            self.hubs[:] = [hub for hub in self.hubs if hub != hub_to_be_disengaged]
        self._hub_lock.release()

    def advertise(self, connection):
        self._hub_lock.acquire()
        for hub in self.hubs:
            hub.advertise(connection)
        self._hub_lock.release()

    def unadvertise(self, connection):
        self._hub_lock.acquire()
        for hub in self.hubs:
            hub.unadvertise(connection)
        self._hub_lock.release()

    def match_remote_gateway_name(self, remote_gateway_name):
        '''
          Parses the hub lists looking for strong (identical) and
          weak (matches the name without the uuid hash) matches.
        '''
        matches = []
        weak_matches = []  # doesn't match any hash names, but matches a base name
        self._hub_lock.acquire()
        for hub in self.hubs:
            matches.extend(hub.matches_remote_gateway_name(remote_gateway_name))
            weak_matches.extend(hub.matches_remote_gateway_basename(remote_gateway_name))
        self._hub_lock.release()
        # these are hash name lists, make sure they didn't pick up matches for a single hash name from multiple hubs
        matches = list(set(matches))
        weak_matches = list(set(weak_matches))
        return matches, weak_matches

    def publish_network_statistics(self, statistics):
        '''
          Publish network statistics to every hub this gateway is connected to.

          @param statistics
          @type gateway_msgs.ConnectionStatistics
        '''
        self._hub_lock.acquire()
        for hub in self.hubs:
            hub.publish_network_statistics(statistics)
        self._hub_lock.release()
