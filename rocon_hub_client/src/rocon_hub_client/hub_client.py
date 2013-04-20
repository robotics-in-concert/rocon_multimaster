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

# local imports
import hub_api
from .exceptions import HubNameNotFoundError, HubNotFoundError, \
                        HubConnectionBlacklistedError, HubConnectionNotWhitelistedError

##############################################################################
# Hub
##############################################################################


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
        self.uri = str(ip) + ":" + str(port)
        self._redis_keys = {}
        self._redis_channels = {}

        # redis server connection
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

        # whitelists, blacklists
        if self.uri in blacklist or self.name in blacklist:
            raise HubConnectionBlacklistedError("ignoring blacklisted hub [%s]" % self.uri)
        if not ((len(whitelist) == 0) or (self.uri in whitelist) or (self.name in whitelist)):
            raise HubConnectionNotWhitelistedError("hub/ip not in non-empty whitelist [%s]%s" % (self.name, whitelist))
