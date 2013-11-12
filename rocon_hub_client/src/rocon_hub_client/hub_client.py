#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_hub_client/LICENSE
#
###############################################################################
# Imports
###############################################################################

import redis
import rospy
from urlparse import urlparse
import rocon_utilities

# local imports
import hub_api
from .exceptions import HubNameNotFoundError, HubNotFoundError, \
                        HubConnectionBlacklistedError, HubConnectionNotWhitelistedError

##############################################################################
# Hub
##############################################################################


class HubConnection(redis.Connection):
    '''
      This might be useful if doing connections with socket timeouts and we
      need special functionality. Not currently using though. Pass in to
      the redis server constructor as connection_class=HubConnection
    '''
    def __init__(self, host='localhost', port=6379, db=0, password=None,
                 socket_timeout=1.0, encoding='utf-8',
                 encoding_errors='strict', decode_responses=False):
        super(HubConnection, self).__init__(host, port, db, password,
                 socket_timeout, encoding,
                 encoding_errors, decode_responses)


class Hub(object):

    def __init__(self, ip, port, whitelist=[], blacklist=[]):
        '''
          @param remote_gateway_request_callbacks : to handle redis responses
          @type list of function pointers (back to GatewaySync class

          @param ip : redis server ip
          @param port : redis server port

          @raise HubNameNotFoundError, HubNotFoundError
        '''
        # variables
        self.ip = ip
        self.port = port
        self.uri = str(ip) + ":" + str(port)
        self._redis_keys = {}
        self._redis_channels = {}

        # This is a temporary try-except block just to ping and see if the address we have here is
        # actually resolvable or it times out. Ideally we want to use socket_timeouts throughout,
        # but that will need modification of the way we handle the RedisListenerThread in
        # gateway_hub.py
        try:
            unused_ping = redis.Redis(host=ip, socket_timeout=0.5, port=port).ping()
            # should check ping result? Typically it just throws the timeout error
        except redis.exceptions.ConnectionError:
            self._redis_server = None
            raise HubNotFoundError("couldn't connect to the redis server")
        try:
            self.pool = redis.ConnectionPool(host=ip, port=port, db=0)
            self._redis_server = redis.Redis(connection_pool=self.pool)
            self._redis_pubsub_server = self._redis_server.pubsub()
            hub_key_name = self._redis_server.get("rocon:hub:name")
            # Be careful, hub_name is None, it means the redis server is
            # found but hub_name not yet set or not set at all.
            if not hub_key_name:
                self._redis_server = None
                raise HubNameNotFoundError("couldn't resolve hub name on the redis server [%s:%s]" % (ip, port))
            else:
                self.name = hub_api.key_base_name(hub_key_name)  # perhaps should store all key names somewhere central
                rospy.logdebug("Gateway : resolved hub name [%s].", self.name)
        except redis.exceptions.ConnectionError:
            self._redis_server = None
            raise HubNotFoundError("couldn't connect to the redis server")

        # whitelists, blacklists - check against uri's hash names and non-uuid names
        uri_blacklist = [urlparse(x).hostname + ':' + str(urlparse(x).port) for x in blacklist if urlparse(x).hostname is not None]
        uri_whitelist = [urlparse(x).hostname + ':' + str(urlparse(x).port) for x in whitelist if urlparse(x).hostname is not None]
        nonuuid_blacklist = [rocon_utilities.gateway_basename(x) for x in blacklist if urlparse(x) is None and rocon_utilities.gateway_basename(x)]
        nonuuid_whitelist = [rocon_utilities.gateway_basename(x) for x in whitelist if urlparse(x) is None and rocon_utilities.gateway_basename(x)]
        if self.uri in uri_blacklist or self.name in blacklist or self.name in nonuuid_blacklist:
            raise HubConnectionBlacklistedError("ignoring blacklisted hub [%s]" % self.uri)
        if self.name in blacklist or self.name in nonuuid_whitelist:
            raise HubConnectionBlacklistedError("ignoring blacklisted hub [%s]" % self.uri)
        if not ((len(whitelist) == 0) or (self.uri in uri_whitelist) or (self.name in whitelist)):
            raise HubConnectionNotWhitelistedError("hub/ip not in non-empty whitelist [%s][%s]%s" % (self.name, self.uri, whitelist))

    def disconnect(self):
        '''
          Kills any open socket connections to the redis server. This is
          necessary if the server is out of range, as the py-redis client
          will hang all open connections indefinitely
        '''
        self._redis_server.connection_pool.disconnect()
    
    def __eq__(self, other):
        return self.uri == other.uri

    def __ne__(self, other):
        return not self.__eq__(other)
