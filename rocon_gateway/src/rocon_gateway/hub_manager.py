#!/usr/bin/env pythonupdate
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway/LICENSE
#
###############################################################################
# Imports
###############################################################################

import threading
import rospy
import gateway_msgs.msg as gateway_msgs

# local imports
import hub_api
from .exceptions import GatewayUnavailableError, \
              HubNameNotFoundError, HubNotFoundError

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
          Send an unflip request to the specified gateway through
          the first common hub that can be found.

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
                # I don't think we need more than one hub's info....
                try:
                    hub.send_unflip_request(remote_gateway_name, remote_rule)
                    self._hub_lock.release()
                    return
                except GatewayUnavailableError:
                    pass  # cycle through the other hubs looking as well.
        self._hub_lock.release()

    ##########################################################################
    # Hub Connections
    ##########################################################################

    def connect_to_hub(self, ip, port):
        '''
          Attempts to make a connection and register the gateway with a hub.

          @param ip
          @param port

          @return an integer indicating error (important for the service call)
          @rtype gateway_msgs.ErrorCodes

          @raise
        '''
        try:
            hub = hub_api.Hub(ip, port)
        except HubNotFoundError:
            return None, gateway_msgs.ErrorCodes.HUB_CONNECTION_UNRESOLVABLE, "couldn't connect to the redis server."
        except HubNameNotFoundError:
            return None, gateway_msgs.ErrorCodes.HUB_NAME_NOT_FOUND, "couldn't resolve hub name on the redis server [%s:%s]" % (ip, port)
        if ip in self._param['hub_blacklist']:
            return None, gateway_msgs.ErrorCodes.HUB_CONNECTION_BLACKLISTED, "ignoring blacklisted hub [%s]" % ip
        elif hub.name in self._param['hub_blacklist']:
            return None, gateway_msgs.ErrorCodes.HUB_CONNECTION_BLACKLISTED, "ignoring blacklisted hub [%s]" % hub.name
        # Handle whitelist (ip or hub name)
        if (len(self._param['hub_whitelist']) == 0) or (ip in self._param['hub_whitelist']) or (hub.name in self._param['hub_whitelist']):
            self._hub_lock.acquire()
            self.hubs.append(hub)
            self._hub_lock.release()
            return hub, gateway_msgs.ErrorCodes.SUCCESS, "success"
        else:
            return None, gateway_msgs.ErrorCodes.HUB_CONNECTION_NOT_IN_NONEMPTY_WHITELIST, "hub/ip not in non-empty whitelist [%s]" % hub.name

    def connect_to_hub_with_timeout(self, ip, port, timeout=rospy.Duration(5.0)):
        '''
          Connect to the hub with a timeout. This is used for direct connections
          where the parameter may be configured by the redis server has fully
          launched and initialised.

          @return hub and error code pair (hub, ErrorCodes.SUCCESS) or (hub, ErrorCodes.xxx)
          @rtype (Hub, gateway_msgs.ErrorCodes.xxx)
        '''
        start_time = rospy.Time.now()
        hub = gateway_msgs.ErrorCodes.HUB_UNKNOWN_ERROR
        error_code = None
        error_code_str = ""
        while not rospy.is_shutdown() and not (rospy.Time.now() - start_time > timeout):
            rospy.loginfo("Gateway : attempting direct connection to hub [%s:%s]" % (ip, port))
            hub, error_code, error_code_str = self.connect_to_hub(ip, port)
            if hub or error_code == gateway_msgs.ErrorCodes.HUB_CONNECTION_BLACKLISTED or \
                      error_code == gateway_msgs.ErrorCodes.HUB_CONNECTION_NOT_IN_NONEMPTY_WHITELIST:
                break
            else:
                rospy.sleep(0.3)
        return hub, error_code, error_code_str

    def disengage_hub(self, hub_to_be_disengaged):
        '''
          Disengages a hub. Make sure all necessary connections
          are cleaned up before calling this (Gateway.disengage_hub).

          @param hub_to_be_disengaged
        '''
        #uri = str(ip) + ":" + str(port)
        # Could dig in and find the name here, but not worth the bother.
        rospy.loginfo("Gateway : lost connection to the hub [%s][%s]" % (hub_to_be_disengaged.name, hub_to_be_disengaged.uri))
        self._hub_lock.acquire()
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
